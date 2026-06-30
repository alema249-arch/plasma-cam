"""
Plasma CAM - Gコードジェネレーター
切断順序: Point-in-polygon で階層検出 → 内側から外側
リードイン: 接線の法線ベクトルで正確な方向を計算
"""
import math
from shapely.geometry import Polygon, Point
from kerf_offset import compute_offset_points, compute_path_types


# ======================================================================
# メインエントリ
# ======================================================================

def _common_settings(settings):
    """設定dictから共通パラメータを取り出す"""
    return {
        'fr':             int(settings.get('feed_rate', 3000)),
        'lead_in_length': float(settings.get('lead_in_length', 5.0)),
        'lead_out_length':float(settings.get('lead_out_length', 0.0)),
        'kerf_width':     float(settings.get('kerf_width', 0.0)),
        # pierce_delay がなければ cold_pierce_ms(ms→s) を流用、それもなければ 0.5s
        'pierce_delay':   float(settings.get('pierce_delay',
                              settings.get('cold_pierce_ms', 500) / 1000.0)),
        'post_cut_delay': float(settings.get('post_cut_delay', 0.0)),
        'pierce_z_height':  float(settings.get('pierce_z_height', 0.0)),
        'cut_z_height':     float(settings.get('cut_z_height', 0.0)),
        'home_z_clearance': float(settings.get('home_z_clearance', 0.0)),
    }


def generate_gcode(dxf_entries: list, settings: dict) -> str:
    p = _common_settings(settings)
    fr             = p['fr']
    lead_in_length = p['lead_in_length']
    lead_out_length= p['lead_out_length']
    kerf_width     = p['kerf_width']
    pierce_delay   = p['pierce_delay']
    post_cut_delay = p['post_cut_delay']

    pierce_z_height  = p['pierce_z_height']
    cut_z_height     = p['cut_z_height']
    home_z_clearance = p['home_z_clearance']
    use_z = pierce_z_height > 0 or cut_z_height > 0

    lines = [
        '; Plasma CAM G-code',
        f'; Feed: {fr} mm/min  Lead-in: {lead_in_length}mm  Kerf: {kerf_width}mm',
        f'; Pierce: {pierce_delay}s  Post-cut: {post_cut_delay}s',
        f'; Pierce Z: {pierce_z_height}mm  Home clearance: {home_z_clearance}mm',
        '', 'G21', 'G90', 'G94', 'M5', '',
    ]
    if use_z:
        _z_up(lines, pierce_z_height)   # 開始時に1回だけ移動高さへ上昇
        lines.append('')

    path_global_idx = 0
    for entry in dxf_entries:
        ox = entry.get('offset_x', 0.0)
        oy = entry.get('offset_y', 0.0)
        paths = entry['paths']

        def px(x): return x + ox
        def py(y): return y + oy

        order, inner_flags = _hierarchical_order(paths, ox, oy)

        for idx in order:
            path     = paths[idx]
            is_inner = inner_flags[idx]
            if not path.segments:
                path_global_idx += 1
                continue

            ptype = '穴' if is_inner else '外形'
            lines.append(f'; --- Path {path_global_idx+1}  {ptype} ---')
            path_global_idx += 1

            if kerf_width > 0 and _path_all_arcs(path):
                # アーク専用: 半径を解析的に補正 → G2/G3 で滑らか
                kd = _arc_kerf_delta(kerf_width, is_inner)
                lead_start = _calc_lead_start_arc_kerf(
                    path, kd, is_inner, lead_in_length, ox, oy)
                _write_arc_kerf_path(lines, path, fr, pierce_delay,
                                     lead_in_length, lead_out_length,
                                     is_inner, px, py, lead_start,
                                     pierce_z_height, kd)
            else:
                offset_pts = None
                if kerf_width > 0 and path.closed:
                    offset_pts = compute_offset_points(path, kerf_width, is_inner=is_inner)
                lead_start = _calc_lead_start(path, offset_pts, is_inner, lead_in_length, ox, oy)
                if offset_pts and len(offset_pts) >= 2:
                    _write_offset_path(lines, offset_pts, fr, pierce_delay,
                                       lead_in_length, lead_out_length,
                                       is_inner, px, py, lead_start,
                                       pierce_z_height, cut_z_height)
                else:
                    _write_original_path(lines, path, fr, pierce_delay,
                                         lead_in_length, lead_out_length,
                                         is_inner, px, py, lead_start,
                                         pierce_z_height, cut_z_height)

            lines.append('M5')
            if use_z:
                _z_up(lines, pierce_z_height)   # 次の移動のため安全高さへ上昇
            if post_cut_delay > 0:
                lines.append(f'G4 P{post_cut_delay:.3f}')
            lines.append('')

    # ホーム退避: ループ終了時点ではすでに pierce_z_height 分上昇済み。
    # そこからさらに (home_z_clearance - pierce_z_height) 分だけ上昇して
    # XY原点へ移動後、home_z_clearance 分下降する。
    # ※一度下げてから上げる不要な二重移動（リミットスイッチ接触リスク）を回避。
    if home_z_clearance > 0:
        extra = home_z_clearance - (pierce_z_height if use_z else 0)
        if extra > 0.001:
            _z_up(lines, extra)
        elif extra < -0.001:
            _z_down(lines, -extra)
    lines.append('G0 X0 Y0')
    if home_z_clearance > 0:
        _z_down(lines, home_z_clearance)
    lines += ['M30']
    return '\n'.join(lines)


# ======================================================================
# cam_plan からGコード生成 (手動順序・リードイン方向対応)
# ======================================================================

def generate_from_plan(plan: list, settings: dict) -> str:
    """
    plan: list of {
        'entry': dxf_entry,
        'path': Path,
        'is_inner': bool,
        'leadin': 'inside' | 'outside',
    }
    """
    p = _common_settings(settings)
    fr             = p['fr']
    lead_in_length = p['lead_in_length']
    lead_out_length= p['lead_out_length']
    kerf_width     = p['kerf_width']
    pierce_delay   = p['pierce_delay']
    post_cut_delay = p['post_cut_delay']

    pierce_z_height  = p['pierce_z_height']
    cut_z_height     = p['cut_z_height']
    home_z_clearance = p['home_z_clearance']
    use_z = pierce_z_height > 0 or cut_z_height > 0

    lines = [
        '; Plasma CAM G-code',
        f'; Feed: {fr} mm/min  Lead-in: {lead_in_length}mm  Kerf: {kerf_width}mm',
        f'; Pierce: {pierce_delay}s  Post-cut: {post_cut_delay}s',
        f'; Pierce Z: {pierce_z_height}mm  Home clearance: {home_z_clearance}mm',
        '', 'G21', 'G90', 'G94', 'M5', '',
    ]
    if use_z:
        _z_up(lines, pierce_z_height)   # 開始時に1回だけ移動高さへ上昇
        lines.append('')

    for i, item in enumerate(plan):
        entry    = item['entry']
        path     = item['path']
        is_inner = item['is_inner']
        leadin   = item.get('leadin', 'inside')
        ox = entry.get('offset_x', 0.0)
        oy = entry.get('offset_y', 0.0)

        def px(x): return x + ox
        def py(y): return y + oy

        ptype = '穴' if is_inner else '外形'
        lines.append(f'; --- Path {i+1}  {ptype}  リードイン:{leadin} ---')

        if kerf_width > 0 and _path_all_arcs(path):
            # アーク専用: 半径を解析的に補正 → G2/G3 で滑らか
            kd = _arc_kerf_delta(kerf_width, is_inner)
            lead_start = _calc_lead_start_arc_kerf(
                path, kd, is_inner, lead_in_length, ox, oy)
            _write_arc_kerf_path(lines, path, fr, pierce_delay,
                                 lead_in_length, lead_out_length,
                                 is_inner, px, py, lead_start,
                                 pierce_z_height, kd)
        else:
            offset_pts = None
            if kerf_width > 0 and path.closed:
                offset_pts = compute_offset_points(path, kerf_width, is_inner=is_inner)
            lead_start = _calc_lead_start_dir(
                path, offset_pts, leadin, lead_in_length, ox, oy)
            if offset_pts and len(offset_pts) >= 2:
                _write_offset_path(lines, offset_pts, fr, pierce_delay,
                                   lead_in_length, lead_out_length,
                                   is_inner, px, py, lead_start,
                                   pierce_z_height, cut_z_height)
            else:
                _write_original_path(lines, path, fr, pierce_delay,
                                     lead_in_length, lead_out_length,
                                     is_inner, px, py, lead_start,
                                     pierce_z_height, cut_z_height)

        lines.append('M5')
        if use_z:
            _z_up(lines, pierce_z_height)   # 次の移動のため安全高さへ上昇
        if post_cut_delay > 0:
            lines.append(f'G4 P{post_cut_delay:.3f}')
        lines.append('')

    if home_z_clearance > 0:
        extra = home_z_clearance - (pierce_z_height if use_z else 0)
        if extra > 0.001:
            _z_up(lines, extra)
        elif extra < -0.001:
            _z_down(lines, -extra)
    lines.append('G0 X0 Y0')
    if home_z_clearance > 0:
        _z_down(lines, home_z_clearance)
    lines += ['M30']
    return '\n'.join(lines)


def _calc_lead_start_dir(path, offset_pts, leadin, lead_len, ox, oy):
    """
    leadin='inside'  → lead_start が輪郭の内側（重心方向）
    leadin='outside' → lead_start が輪郭の外側（重心逆方向）
    """
    if lead_len <= 0:
        return None

    if offset_pts and len(offset_pts) >= 2:
        sx, sy  = offset_pts[0]
        all_pts = offset_pts
    else:
        sx, sy  = path.segments[0].start
        all_pts = path.get_display_points()

    if not all_pts:
        return (sx + ox, sy + oy)

    cx = sum(p[0] for p in all_pts) / len(all_pts)
    cy = sum(p[1] for p in all_pts) / len(all_pts)
    dx, dy = cx - sx, cy - sy
    d = math.hypot(dx, dy)
    if d < 1e-10:
        return (sx + ox, sy + oy)

    if leadin == 'inside':
        # 重心方向（内側）。穴サイズを超えないよう 85% に制限
        eff = min(lead_len, d * 0.85)
        nx, ny = dx/d * eff, dy/d * eff
    else:
        # 重心逆方向（外側）
        nx, ny = -dx/d * lead_len, -dy/d * lead_len

    return (sx + nx + ox, sy + ny + oy)


# ======================================================================
# 切断順序: 階層検出 + 最近隣
# ======================================================================

def _hierarchical_order(paths, ox, oy):
    """
    ポリゴン同士の包含関係で入れ子の深さを判定する。
    深い(内側に多く包まれている)ものほど先に切断。
    同じ深さは最近隣法で並べる。

    深さは「パスiの重心を他のポリゴンが含むか」ではなく、
    「他のポリゴン全体がパスiを完全に包含するか」で判定する。
    同心円のように複数のパスの重心が一致するケースでは、
    重心だけを見ると大小関係を区別できず深さ計算が壊れるため。
    """
    polys = []
    for p in paths:
        pts = p.get_display_points()
        if len(pts) < 3:
            polys.append(None)
            continue
        try:
            shifted = [(x+ox, y+oy) for x,y in pts]
            poly = Polygon(shifted)
            if not poly.is_valid:
                poly = poly.buffer(0)
            polys.append(poly if not poly.is_empty else None)
        except Exception:
            polys.append(None)

    n = len(paths)
    depths = []
    for i in range(n):
        if polys[i] is None:
            depths.append(0)
            continue
        depth = sum(
            1 for j in range(n)
            if j != i and polys[j] is not None
            and polys[j].contains(polys[i])
        )
        depths.append(depth)

    # is_inner: 奇数深さ = 穴, 偶数(1以上) = 外形内ネスト
    inner_flags = [d % 2 == 1 for d in depths]

    def centroid_xy(idx):
        pts = paths[idx].get_display_points()
        if not pts:
            return (0.0, 0.0)
        return (sum(x for x,y in pts)/len(pts) + ox,
                sum(y for x,y in pts)/len(pts) + oy)

    def nearest_neighbor(idxs, start_pos=None):
        if not idxs:
            return []
        rem = list(idxs)
        if start_pos is None:
            result = [rem.pop(0)]
        else:
            best = min(rem, key=lambda i: math.hypot(
                centroid_xy(i)[0]-start_pos[0],
                centroid_xy(i)[1]-start_pos[1]))
            result = [best]
            rem.remove(best)
        while rem:
            last = centroid_xy(result[-1])
            best = min(rem, key=lambda i: math.hypot(
                centroid_xy(i)[0]-last[0],
                centroid_xy(i)[1]-last[1]))
            result.append(best)
            rem.remove(best)
        return result

    max_depth = max(depths) if depths else 0
    order = []
    last_pos = None
    for depth in range(max_depth, -1, -1):
        group = [i for i,d in enumerate(depths) if d == depth]
        if not group:
            continue
        ordered = nearest_neighbor(group, last_pos)
        order.extend(ordered)
        if ordered:
            last_pos = centroid_xy(ordered[-1])

    return order, inner_flags


# ======================================================================
# リードイン始点: 法線ベクトル方式
# ======================================================================

def _calc_lead_start(path, offset_pts, is_inner, lead_len, ox, oy):
    """
    リードイン始点を計算する。

    【ルール】全パス共通:
      lead_start = カット開始点 + 重心方向(内向き) * lead_len

    ・穴  → 穴の内側(捨て材)に pierce → エッジが綺麗
    ・外形 → 製品内側に pierce → 外周エッジに傷なし
    """
    if lead_len <= 0:
        return None

    # カット開始点
    if offset_pts and len(offset_pts) >= 2:
        sx, sy  = offset_pts[0]
        all_pts = offset_pts
    else:
        sx, sy  = path.segments[0].start
        all_pts = path.get_display_points()

    # 重心を計算
    if all_pts:
        cx = sum(p[0] for p in all_pts) / len(all_pts)
        cy = sum(p[1] for p in all_pts) / len(all_pts)
    else:
        return (sx + ox, sy + oy)

    # 重心→開始点 の方向（内向き単位ベクトル）
    dx = cx - sx
    dy = cy - sy
    d  = math.hypot(dx, dy)
    if d < 1e-10:
        return (sx + ox, sy + oy)

    if is_inner:
        # 穴: 重心方向（内側にピアス）
        # リードイン長が穴サイズを超えると外に出るため 85% に自動制限
        eff = min(lead_len, d * 0.85)
        nx, ny = dx / d * eff, dy / d * eff
    else:
        # 外形: 重心の逆方向（ワーク外側にピアス）
        nx, ny = -dx / d * lead_len, -dy / d * lead_len

    return (sx + nx + ox, sy + ny + oy)


def _path_all_arcs(path) -> bool:
    """パスが全てアークセグメントで構成されているか（円・円弧など）"""
    return path.closed and bool(path.segments) and all(
        seg.type == 'arc' for seg in path.segments)


def _arc_kerf_delta(kerf_width: float, is_inner: bool) -> float:
    """アーク用カーフ補正量: 内側(穴)なら縮小、外側なら拡大"""
    return -(kerf_width / 2) if is_inner else (kerf_width / 2)


def _calc_lead_start_arc_kerf(path, kerf_delta: float, is_inner: bool,
                               lead_len: float, ox: float, oy: float):
    """アークカーフ補正パス用のリードイン始点を計算"""
    if lead_len <= 0 or not path.segments:
        return None
    seg0 = path.segments[0]
    cx, cy, r, sa, ea, ccw = seg0.data
    r_new = r + kerf_delta
    if r_new <= 0:
        return None
    sa_r = math.radians(sa)
    sx = cx + r_new * math.cos(sa_r)
    sy = cy + r_new * math.sin(sa_r)
    # 重心 = アーク中心に近似
    dx, dy = cx - sx, cy - sy
    d = math.hypot(dx, dy)
    if d < 1e-10:
        return (sx + ox, sy + oy)
    if is_inner:
        eff = min(lead_len, d * 0.85)
        nx, ny = dx / d * eff, dy / d * eff
    else:
        nx, ny = -dx / d * lead_len, -dy / d * lead_len
    return (sx + nx + ox, sy + ny + oy)


def _emit_arc_segment(lines, cur_pos, cx, cy, r, sa, ea, ccw, fr, px, py,
                       tol=0.001):
    """
    1個のアークセグメントをG2/G3で出力する。
    cur_pos: 直前に実際に出力した(オフセット適用後・丸め済み)座標 (cur_x, cur_y)。
    cx, cy, r, sa, ea: オフセット適用前のローカル円弧パラメータ（角度は度）。
    始点〜中心の距離と終点〜中心の距離の差が tol(mm) を超える場合は
    GRBLのerror:8(円弧誤差超過)を避けるため、G1直線近似にフォールバックする。
    戻り値: 新しい cur_pos
    """
    cur_x, cur_y = cur_pos
    if r <= 1e-9:
        return cur_pos

    ea_r = math.radians(ea)
    end_x = px(cx + r * math.cos(ea_r))
    end_y = py(cy + r * math.sin(ea_r))
    center_x = px(cx)
    center_y = py(cy)

    # GRBLに実際に送る値（.3fで丸めた値）でI/Jを計算し検証する。
    # GRBLはこの丸め後の値を使って始点〜中心と終点〜中心の距離を比較するため、
    # 丸め前の浮動小数で検証すると誤判定が起きる。
    ix_g = round(center_x - cur_x, 3)
    iy_g = round(center_y - cur_y, 3)
    ex_r = round(end_x, 3)
    ey_r = round(end_y, 3)

    r1 = math.hypot(ix_g, iy_g)                                 # GRBLが計算する始点〜中心
    r2 = math.hypot(ex_r - (cur_x + ix_g), ey_r - (cur_y + iy_g))  # GRBLが計算する終点〜中心

    if r1 > 1e-6 and abs(r1 - r2) <= tol:
        cmd = 'G3' if ccw else 'G2'
        lines.append(
            f'{cmd} X{ex_r:.3f} Y{ey_r:.3f} I{ix_g:.3f} J{iy_g:.3f} F{fr}')
        return (ex_r, ey_r)

    # フォールバック: 円弧誤差が許容値を超えるためG1で細分化して近似する
    return _emit_arc_as_lines(lines, cur_x, cur_y, cx, cy, r, sa, ea, ccw,
                               fr, px, py, tol)


def _emit_arc_as_lines(lines, cur_x, cur_y, cx, cy, r, sa, ea, ccw, fr,
                        px, py, tol=0.001):
    """円弧をG1直線近似に分割して出力するフォールバック処理"""
    sa_r = math.radians(sa)
    ea_r = math.radians(ea)
    if ccw:
        dtheta = ea_r - sa_r
        if dtheta <= 0:
            dtheta += 2 * math.pi
    else:
        dtheta = sa_r - ea_r
        if dtheta <= 0:
            dtheta += 2 * math.pi

    if r <= 1e-9 or dtheta <= 1e-9:
        return (cur_x, cur_y)

    # 弦のサジッタ(sagitta) <= tol になるよう分割数を決定
    ratio = max(0.0, min(1.0, 1 - tol / r))
    max_step = 2 * math.acos(ratio) if ratio < 1.0 else math.pi / 18
    if max_step <= 1e-6:
        max_step = math.pi / 18
    steps = max(2, int(math.ceil(dtheta / max_step)))

    last = (cur_x, cur_y)
    for i in range(1, steps + 1):
        t = sa_r + (dtheta * i / steps) * (1 if ccw else -1)
        x = round(px(cx + r * math.cos(t)), 3)
        y = round(py(cy + r * math.sin(t)), 3)
        lines.append(f'G1 X{x:.3f} Y{y:.3f} F{fr}')
        last = (x, y)
    return last


def _write_arc_kerf_path(lines, path, fr, pierce, lead_in, lead_out,
                          is_inner, px, py, lead_start,
                          pierce_z=0.0, kerf_delta=0.0):
    """
    アークパス専用カーフ補正Gコード生成。
    半径を解析的に ±kerf/2 調整して G2/G3 コマンドを出力。
    Shapely 近似なし → 真円弧のまま滑らかに切断できる。
    円弧誤差がGRBLの許容値を超える場合はG1近似にフォールバックする。
    """
    use_z = pierce_z > 0

    # 最初のセグメントの補正済み始点
    seg0 = path.segments[0]
    cx0, cy0, r0, sa0, _, _ = seg0.data
    r_new0 = r0 + kerf_delta
    if r_new0 <= 0:
        return
    sa0_r = math.radians(sa0)
    start_x = cx0 + r_new0 * math.cos(sa0_r)
    start_y = cy0 + r_new0 * math.sin(sa0_r)

    if lead_start is not None:
        lines.append(f'G0 X{lead_start[0]:.3f} Y{lead_start[1]:.3f}')
        lines.append('M3')
        lines.append(f'G4 P{pierce:.3f}')
        if use_z:
            _z_down(lines, pierce_z)
        lines.append(f'G1 X{px(start_x):.3f} Y{py(start_y):.3f} F{fr}')
    else:
        lines.append(f'G0 X{px(start_x):.3f} Y{py(start_y):.3f}')
        lines.append('M3')
        lines.append(f'G4 P{pierce:.3f}')
        if use_z:
            _z_down(lines, pierce_z)

    cur_pos = (round(px(start_x), 3), round(py(start_y), 3))

    # 各アークセグメントを G2/G3 で出力（誤差超過時はG1にフォールバック）
    for seg in path.segments:
        cx, cy, r, sa, ea, ccw = seg.data
        r_new = r + kerf_delta
        if r_new <= 0:
            continue
        cur_pos = _emit_arc_segment(lines, cur_pos, cx, cy, r_new, sa, ea,
                                     ccw, fr, px, py)

    # リードアウト
    if lead_out > 0:
        last = path.segments[-1]
        dx, dy = _tangent_at_end(last)
        d = math.hypot(dx, dy)
        if d > 1e-10:
            last_x, last_y = cur_pos
            lines.append(
                f'G1 X{last_x + dx/d*lead_out:.3f} '
                f'Y{last_y + dy/d*lead_out:.3f} F{fr}')


def _tangent_at_start(seg) -> tuple:
    """セグメント始点での接線ベクトル(未正規化)"""
    if seg.type == 'line':
        return (seg.end[0]-seg.start[0], seg.end[1]-seg.start[1])
    cx, cy, r, sa, ea, ccw = seg.data
    sa_r = math.radians(sa)
    if ccw:
        return (-math.sin(sa_r), math.cos(sa_r))
    else:
        return (math.sin(sa_r), -math.cos(sa_r))


# ======================================================================
# Gコード書き出し
# ======================================================================

def _z_up(lines, dz):
    """現在位置から相対的にZ軸を上昇させる"""
    lines.append('G91')
    lines.append(f'G0 Z{dz:.3f}')
    lines.append('G90')


def _z_down(lines, dz):
    """現在位置から相対的にZ軸を下降させる（切断高さに戻る）"""
    lines.append('G91')
    lines.append(f'G0 Z-{dz:.3f}')
    lines.append('G90')


def _write_offset_path(lines, offset_pts, fr, pierce,
                       lead_in, lead_out, is_inner, px, py, lead_start,
                       pierce_z=0.0, cut_z=0.0):
    sx, sy = offset_pts[0]
    use_z = pierce_z > 0

    if lead_start is not None:
        lines.append(f'G0 X{lead_start[0]:.3f} Y{lead_start[1]:.3f}')
        lines.append('M3')
        lines.append(f'G4 P{pierce:.3f}')
        if use_z:
            _z_down(lines, pierce_z)      # 切断高さへ下降
        lines.append(f'G1 X{px(sx):.3f} Y{py(sy):.3f} F{fr}')
    else:
        lines.append(f'G0 X{px(sx):.3f} Y{py(sy):.3f}')
        lines.append('M3')
        lines.append(f'G4 P{pierce:.3f}')
        if use_z:
            _z_down(lines, pierce_z)

    for pt in offset_pts[1:]:
        lines.append(f'G1 X{px(pt[0]):.3f} Y{py(pt[1]):.3f} F{fr}')

    # リードアウト
    if lead_out > 0 and len(offset_pts) >= 2:
        dx = offset_pts[-1][0]-offset_pts[-2][0]
        dy = offset_pts[-1][1]-offset_pts[-2][1]
        d  = math.hypot(dx, dy)
        if d > 1e-10:
            ex = px(offset_pts[-1][0] + dx/d*lead_out)
            ey = py(offset_pts[-1][1] + dy/d*lead_out)
            lines.append(f'G1 X{ex:.3f} Y{ey:.3f} F{fr}')
            return (ex, ey)

    return (px(offset_pts[-1][0]), py(offset_pts[-1][1]))


def _write_original_path(lines, path, fr, pierce,
                         lead_in, lead_out, is_inner, px, py, lead_start,
                         pierce_z=0.0, cut_z=0.0):
    start = path.segments[0].start
    use_z = pierce_z > 0

    if lead_start is not None:
        lines.append(f'G0 X{lead_start[0]:.3f} Y{lead_start[1]:.3f}')
        lines.append('M3')
        lines.append(f'G4 P{pierce:.3f}')
        if use_z:
            _z_down(lines, pierce_z)      # 切断高さへ下降
        lines.append(f'G1 X{px(start[0]):.3f} Y{py(start[1]):.3f} F{fr}')
    else:
        lines.append(f'G0 X{px(start[0]):.3f} Y{py(start[1]):.3f}')
        lines.append('M3')
        lines.append(f'G4 P{pierce:.3f}')
        if use_z:
            _z_down(lines, pierce_z)

    cur_pos = (round(px(start[0]), 3), round(py(start[1]), 3))

    for seg in path.segments:
        if seg.type == 'line':
            ex = round(px(seg.end[0]), 3)
            ey = round(py(seg.end[1]), 3)
            lines.append(f'G1 X{ex:.3f} Y{ey:.3f} F{fr}')
            cur_pos = (ex, ey)
        elif seg.type == 'arc':
            cx, cy, r, sa, ea, ccw = seg.data
            cur_pos = _emit_arc_segment(lines, cur_pos, cx, cy, r, sa, ea,
                                         ccw, fr, px, py)

    if lead_out > 0:
        dx, dy = _tangent_at_end(path.segments[-1])
        d = math.hypot(dx, dy)
        if d > 1e-10:
            last_x, last_y = cur_pos
            ex = last_x + dx/d*lead_out
            ey = last_y + dy/d*lead_out
            lines.append(f'G1 X{ex:.3f} Y{ey:.3f} F{fr}')
            return (ex, ey)

    return cur_pos


def _tangent_at_end(seg) -> tuple:
    if seg.type == 'line':
        return (seg.end[0]-seg.start[0], seg.end[1]-seg.start[1])
    cx, cy, r, sa, ea, ccw = seg.data
    ea_r = math.radians(ea)
    if ccw:
        return (-math.sin(ea_r), math.cos(ea_r))
    else:
        return (math.sin(ea_r), -math.cos(ea_r))
