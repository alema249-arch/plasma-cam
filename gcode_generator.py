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

def generate_gcode(dxf_entries: list, settings: dict) -> str:
    feed_rate       = settings.get('feed_rate', 3000)
    lead_in_length  = settings.get('lead_in_length', 5.0)
    lead_in_type    = settings.get('lead_in_type', 'line')
    lead_out_length = settings.get('lead_out_length', 0.0)
    kerf_width      = settings.get('kerf_width', 0.0)
    hot_dist        = float(settings.get('hot_start_distance', 50.0))
    hot_time        = float(settings.get('hot_start_time', 2.5))
    hot_pierce      = float(settings.get('hot_pierce_ms',  800))  / 1000.0
    cold_pierce     = float(settings.get('cold_pierce_ms', 2400)) / 1000.0
    rapid_speed     = float(settings.get('rapid_speed', 5000.0))

    fr = int(feed_rate)

    lines = [
        '; Plasma CAM G-code',
        f'; Feed: {fr} mm/min  Lead-in: {lead_in_length}mm  Kerf: {kerf_width}mm',
        '',
        'G21', 'G90', 'G94', 'M5',
        'G0 X0 Y0', '',
    ]

    prev_end = None
    path_global_idx = 0

    for entry in dxf_entries:
        ox = entry.get('offset_x', 0.0)
        oy = entry.get('offset_y', 0.0)
        paths = entry['paths']

        def px(x): return x + ox
        def py(y): return y + oy

        # 階層的切断順序 (内側→外側)
        order, inner_flags = _hierarchical_order(paths, ox, oy)

        for idx in order:
            path     = paths[idx]
            is_inner = inner_flags[idx]

            if not path.segments:
                path_global_idx += 1
                continue

            offset_pts = None
            if kerf_width > 0 and path.closed:
                offset_pts = compute_offset_points(
                    path, kerf_width, is_inner=is_inner)

            # リードイン開始点 (法線ベクトル方式)
            lead_start = _calc_lead_start(
                path, offset_pts, is_inner, lead_in_length, ox, oy)

            # スマートピアシング判定
            if prev_end is None or lead_start is None:
                pierce = cold_pierce
                label  = 'コールドスタート'
            else:
                d   = math.hypot(lead_start[0]-prev_end[0],
                                 lead_start[1]-prev_end[1])
                t   = d / (rapid_speed / 60.0)
                if d <= hot_dist and t <= hot_time:
                    pierce = hot_pierce
                    label  = f'ホット d={d:.0f}mm'
                else:
                    pierce = cold_pierce
                    label  = f'コールド d={d:.0f}mm'

            lines.append(
                f'; --- Path {path_global_idx+1}  '
                f'{"穴" if is_inner else "外形"}  [{label}] ---')
            path_global_idx += 1

            if offset_pts and len(offset_pts) >= 2:
                end = _write_offset_path(
                    lines, offset_pts, fr, pierce,
                    lead_in_length, lead_out_length,
                    is_inner, px, py, lead_start)
            else:
                end = _write_original_path(
                    lines, path, fr, pierce,
                    lead_in_length, lead_out_length,
                    is_inner, px, py, lead_start)

            lines.append('M5')
            lines.append('')
            prev_end = end

    lines += ['G0 X0 Y0', 'M30']
    return '\n'.join(lines)


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
    カット開始点での接線→法線を計算し、
    適切な方向（内側or外側）にリードイン始点を置く。

    穴(is_inner=True) : 法線の「重心向き」= 穴内部(捨て材) から入る
    外形(is_inner=False): 法線の「重心逆向き」= 輪郭外側(捨て材) から入る
    """
    if lead_len <= 0:
        return None

    # カット開始点と接線ベクトル
    if offset_pts and len(offset_pts) >= 2:
        sx, sy   = offset_pts[0]
        tx, ty   = offset_pts[1][0]-offset_pts[0][0], \
                   offset_pts[1][1]-offset_pts[0][1]
        all_pts  = offset_pts
    else:
        seg = path.segments[0]
        sx, sy = seg.start
        tx, ty = _tangent_at_start(seg)
        all_pts = path.get_display_points()

    # 接線を正規化
    td = math.hypot(tx, ty)
    if td < 1e-10:
        tx, ty = 1.0, 0.0
    else:
        tx, ty = tx/td, ty/td

    # 左法線 (CCW回転), 右法線 (CW回転)
    lnx, lny =  -ty,  tx   # 左法線
    rnx, rny =   ty, -tx   # 右法線

    # 重心を計算
    if all_pts:
        cx = sum(p[0] for p in all_pts) / len(all_pts)
        cy = sum(p[1] for p in all_pts) / len(all_pts)
    else:
        cx, cy = sx, sy

    # 重心方向ベクトル
    gcx, gcy = cx - sx, cy - sy
    gd = math.hypot(gcx, gcy)
    if gd < 1e-10:
        gcx, gcy = 0.0, 1.0
    else:
        gcx, gcy = gcx/gd, gcy/gd

    # 左法線・右法線どちらが重心向きか
    left_dot  = lnx*gcx + lny*gcy
    right_dot = rnx*gcx + rny*gcy

    if left_dot >= right_dot:
        inward_nx,  inward_ny  = lnx,  lny
        outward_nx, outward_ny = rnx,  rny
    else:
        inward_nx,  inward_ny  = rnx,  rny
        outward_nx, outward_ny = lnx,  lny

    if is_inner:
        # 穴: 重心方向(穴内部=捨て材)にオフセット → 内側から入る
        offx, offy = inward_nx * lead_len, inward_ny * lead_len
    else:
        # 外形: 重心逆方向(輪郭外側=捨て材)にオフセット → 外側から入る
        offx, offy = outward_nx * lead_len, outward_ny * lead_len

    return (sx + offx + ox, sy + offy + oy)


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
