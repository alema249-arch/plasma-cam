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
        'pierce_delay':   float(settings.get('pierce_delay', 0.5)),
        'post_cut_delay': float(settings.get('post_cut_delay', 0.0)),
    }


def generate_gcode(dxf_entries: list, settings: dict) -> str:
    p = _common_settings(settings)
    fr             = p['fr']
    lead_in_length = p['lead_in_length']
    lead_out_length= p['lead_out_length']
    kerf_width     = p['kerf_width']
    pierce_delay   = p['pierce_delay']
    post_cut_delay = p['post_cut_delay']

    lines = [
        '; Plasma CAM G-code',
        f'; Feed: {fr} mm/min  Lead-in: {lead_in_length}mm  Kerf: {kerf_width}mm',
        f'; Pierce: {pierce_delay}s  Post-cut: {post_cut_delay}s',
        '', 'G21', 'G90', 'G94', 'M5', 'G0 X0 Y0', '',
    ]

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

            offset_pts = None
            if kerf_width > 0 and path.closed:
                offset_pts = compute_offset_points(path, kerf_width, is_inner=is_inner)

            lead_start = _calc_lead_start(path, offset_pts, is_inner, lead_in_length, ox, oy)

            ptype = '穴' if is_inner else '外形'
            lines.append(f'; --- Path {path_global_idx+1}  {ptype} ---')
            path_global_idx += 1

            if offset_pts and len(offset_pts) >= 2:
                _write_offset_path(lines, offset_pts, fr, pierce_delay,
                                   lead_in_length, lead_out_length,
                                   is_inner, px, py, lead_start)
            else:
                _write_original_path(lines, path, fr, pierce_delay,
                                     lead_in_length, lead_out_length,
                                     is_inner, px, py, lead_start)

            lines.append('M5')
            if post_cut_delay > 0:
                lines.append(f'G4 P{post_cut_delay:.3f}')
            lines.append('')

    lines += ['G0 X0 Y0', 'M30']
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

    lines = [
        '; Plasma CAM G-code',
        f'; Feed: {fr} mm/min  Lead-in: {lead_in_length}mm  Kerf: {kerf_width}mm',
        f'; Pierce: {pierce_delay}s  Post-cut: {post_cut_delay}s',
        '', 'G21', 'G90', 'G94', 'M5', 'G0 X0 Y0', '',
    ]

    for i, item in enumerate(plan):
        entry    = item['entry']
        path     = item['path']
        is_inner = item['is_inner']
        leadin   = item.get('leadin', 'inside')
        ox = entry.get('offset_x', 0.0)
        oy = entry.get('offset_y', 0.0)

        def px(x): return x + ox
        def py(y): return y + oy

        offset_pts = None
        if kerf_width > 0 and path.closed:
            offset_pts = compute_offset_points(path, kerf_width, is_inner=is_inner)

        lead_start = _calc_lead_start_dir(
            path, offset_pts, leadin, lead_in_length, ox, oy)

        ptype = '穴' if is_inner else '外形'
        lines.append(f'; --- Path {i+1}  {ptype}  リードイン:{leadin} ---')

        if offset_pts and len(offset_pts) >= 2:
            _write_offset_path(lines, offset_pts, fr, pierce_delay,
                               lead_in_length, lead_out_length,
                               is_inner, px, py, lead_start)
        else:
            _write_original_path(lines, path, fr, pierce_delay,
                                 lead_in_length, lead_out_length,
                                 is_inner, px, py, lead_start)

        lines.append('M5')
        if post_cut_delay > 0:
            lines.append(f'G4 P{post_cut_delay:.3f}')
        lines.append('')

    lines += ['G0 X0 Y0', 'M30']
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
        # 重心方向（内側）
        nx, ny = dx/d * lead_len, dy/d * lead_len
    else:
        # 重心逆方向（外側）
        nx, ny = -dx/d * lead_len, -dy/d * lead_len

    return (sx + nx + ox, sy + ny + oy)


# ======================================================================
# 切断順序: 階層検出 + 最近隣
# ======================================================================

def _hierarchical_order(paths, ox, oy):
    """
    Point-in-polygon で包含深さを計算。
    深い(内側に多く包まれている)ものほど先に切断。
    同じ深さは最近隣法で並べる。
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
        centroid = polys[i].centroid
        depth = sum(
            1 for j in range(n)
            if j != i and polys[j] is not None
            and polys[j].contains(centroid)
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

    # lead_start = 開始点から重心方向へ lead_len 移動
    nx = dx / d * lead_len
    ny = dy / d * lead_len

    return (sx + nx + ox, sy + ny + oy)


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

def _write_offset_path(lines, offset_pts, fr, pierce,
                       lead_in, lead_out, is_inner, px, py, lead_start):
    sx, sy = offset_pts[0]

    if lead_start is not None:
        lines.append(f'G0 X{lead_start[0]:.3f} Y{lead_start[1]:.3f}')
        lines.append('M3')
        lines.append(f'G4 P{pierce:.3f}')
        lines.append(f'G1 X{px(sx):.3f} Y{py(sy):.3f} F{fr}')
    else:
        lines.append(f'G0 X{px(sx):.3f} Y{py(sy):.3f}')
        lines.append('M3')
        lines.append(f'G4 P{pierce:.3f}')

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
                         lead_in, lead_out, is_inner, px, py, lead_start):
    start = path.segments[0].start

    if lead_start is not None:
        lines.append(f'G0 X{lead_start[0]:.3f} Y{lead_start[1]:.3f}')
        lines.append('M3')
        lines.append(f'G4 P{pierce:.3f}')
        lines.append(f'G1 X{px(start[0]):.3f} Y{py(start[1]):.3f} F{fr}')
    else:
        lines.append(f'G0 X{px(start[0]):.3f} Y{py(start[1]):.3f}')
        lines.append('M3')
        lines.append(f'G4 P{pierce:.3f}')

    for seg in path.segments:
        if seg.type == 'line':
            lines.append(
                f'G1 X{px(seg.end[0]):.3f} Y{py(seg.end[1]):.3f} F{fr}')
        elif seg.type == 'arc':
            cx, cy, r, sa, ea, ccw = seg.data
            ix = cx - seg.start[0]
            iy = cy - seg.start[1]
            cmd = 'G3' if ccw else 'G2'
            lines.append(
                f'{cmd} X{px(seg.end[0]):.3f} Y{py(seg.end[1]):.3f} '
                f'I{ix:.3f} J{iy:.3f} F{fr}')

    last = path.segments[-1].end
    if lead_out > 0:
        dx, dy = _tangent_at_end(path.segments[-1])
        d = math.hypot(dx, dy)
        if d > 1e-10:
            ex = px(last[0] + dx/d*lead_out)
            ey = py(last[1] + dy/d*lead_out)
            lines.append(f'G1 X{ex:.3f} Y{ey:.3f} F{fr}')
            return (ex, ey)

    return (px(last[0]), py(last[1]))


def _tangent_at_end(seg) -> tuple:
    if seg.type == 'line':
        return (seg.end[0]-seg.start[0], seg.end[1]-seg.start[1])
    cx, cy, r, sa, ea, ccw = seg.data
    ea_r = math.radians(ea)
    if ccw:
        return (-math.sin(ea_r), math.cos(ea_r))
    else:
        return (math.sin(ea_r), -math.cos(ea_r))
