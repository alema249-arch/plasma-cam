import math
from typing import List
from dxf_reader import Path
from kerf_offset import compute_offset_points, compute_path_types


def generate_gcode(dxf_entries: list, settings: dict) -> str:
    """
    dxf_entries: list of {'paths': [...], 'offset_x': float, 'offset_y': float, 'name': str}
    スマートピアシング: 前回カット終了位置からの距離・時間でホット/コールドを自動判定。
    """
    feed_rate     = settings.get('feed_rate', 3000)
    lead_in_length  = settings.get('lead_in_length', 5.0)
    lead_in_type    = settings.get('lead_in_type', 'line')
    lead_out_length = settings.get('lead_out_length', 0.0)
    kerf_width      = settings.get('kerf_width', 0.0)

    # スマートピアシング設定
    hot_dist    = float(settings.get('hot_start_distance', 50.0))
    hot_time    = float(settings.get('hot_start_time', 2.5))
    hot_pierce  = float(settings.get('hot_pierce_ms',  800))  / 1000.0  # → 秒
    cold_pierce = float(settings.get('cold_pierce_ms', 2400)) / 1000.0  # → 秒
    rapid_speed = float(settings.get('rapid_speed', 5000.0))  # mm/min

    fr = int(feed_rate)

    lines = [
        '; Plasma CAM G-code  (スマートピアシング対応)',
        f'; Feed rate      : {fr} mm/min',
        f'; Lead-in        : {lead_in_length} mm ({lead_in_type})',
        f'; Lead-out       : {lead_out_length} mm',
        f'; Kerf width     : {kerf_width} mm',
        f'; Hot pierce     : {int(hot_pierce*1000)} ms  (距離≦{hot_dist}mm かつ 時間≦{hot_time}s)',
        f'; Cold pierce    : {int(cold_pierce*1000)} ms',
        f'; Rapid speed    : {int(rapid_speed)} mm/min (時間計算用)',
        '',
        'G21', 'G90', 'G94',
        'M5',
        'G0 X0 Y0',
        '',
    ]

    prev_end = None        # 前回カット終了点（絶対座標）
    path_global_idx = 0

    for entry in dxf_entries:
        ox = entry.get('offset_x', 0.0)
        oy = entry.get('offset_y', 0.0)
        paths = entry['paths']

        def px(x): return x + ox
        def py(y): return y + oy

        # 内外判定
        inner_flags = compute_path_types(paths)

        def _centroid(p):
            pts = p.get_display_points()
            if not pts:
                return (0.0, 0.0)
            return (sum(x for x,y in pts)/len(pts),
                    sum(y for x,y in pts)/len(pts))

        # 穴グループ → 最近隣法で順序最適化、外周グループ → 面積大きい順で最後
        inner_idxs = [i for i,f in enumerate(inner_flags) if f]
        outer_idxs = [i for i,f in enumerate(inner_flags) if not f]

        # 穴を最近隣順に並べる（短い移動で連続切断）
        def _nearest_neighbor(idxs):
            if not idxs:
                return []
            remaining = list(idxs)
            result = [remaining.pop(0)]
            while remaining:
                last_c = _centroid(paths[result[-1]])
                nearest = min(remaining,
                              key=lambda i: math.hypot(
                                  _centroid(paths[i])[0] - last_c[0],
                                  _centroid(paths[i])[1] - last_c[1]))
                result.append(nearest)
                remaining.remove(nearest)
            return result

        # 外周は面積大きい順（最外周が最後）
        def _path_area(p):
            pts = p.get_display_points()
            if len(pts) < 3:
                return 0.0
            n = len(pts)
            return abs(sum(
                pts[i][0]*pts[(i+1)%n][1] - pts[(i+1)%n][0]*pts[i][1]
                for i in range(n))) / 2.0

        outer_idxs_sorted = sorted(outer_idxs, key=lambda i: _path_area(paths[i]))

        order = _nearest_neighbor(inner_idxs) + outer_idxs_sorted
        paths_sorted = [paths[i]       for i in order]
        flags_sorted = [inner_flags[i] for i in order]

        for i, path in enumerate(paths_sorted):
            is_inner = flags_sorted[i]
            if not path.segments:
                path_global_idx += 1
                continue

            offset_pts = None
            if kerf_width > 0 and path.closed:
                offset_pts = compute_offset_points(path, kerf_width, is_inner=is_inner)

            # G0ターゲット（リードイン開始点）と切断終了点を計算
            g0_abs, end_abs = _path_endpoints(path, offset_pts, lead_in_length,
                                               lead_out_length, ox, oy,
                                               is_inner=is_inner)

            # ピアシング時間の自動判定
            if prev_end is None:
                pierce = cold_pierce
                start_label = 'コールドスタート（初回）'
            else:
                dist = math.hypot(g0_abs[0] - prev_end[0], g0_abs[1] - prev_end[1])
                travel_sec = dist / (rapid_speed / 60.0)
                if dist <= hot_dist and travel_sec <= hot_time:
                    pierce = hot_pierce
                    start_label = f'ホットスタート  d={dist:.1f}mm  t={travel_sec:.2f}s'
                else:
                    pierce = cold_pierce
                    start_label = f'コールドスタート  d={dist:.1f}mm  t={travel_sec:.2f}s'

            lines.append(f'; --- Path {path_global_idx + 1} ({entry["name"]})  [{start_label}] ---')
            path_global_idx += 1

            if offset_pts and len(offset_pts) >= 2:
                _generate_offset_path(lines, offset_pts, fr, pierce,
                                      lead_in_length, lead_out_length, px, py,
                                      is_inner=is_inner)
            else:
                _generate_original_path(lines, path, fr, pierce,
                                        lead_in_length, lead_out_length, px, py,
                                        is_inner=is_inner)

            lines.append('M5 ; torch off')
            lines.append('')
            prev_end = end_abs

    lines += ['G0 X0 Y0', 'M30 ; program end']
    return '\n'.join(lines)


def _path_endpoints(path, offset_pts, lead_in_length, lead_out_length,
                    ox, oy, is_inner=False):
    """G0ターゲット（リードイン開始）と切断終了点を絶対座標で返す。"""
    def _px(x): return x + ox
    def _py(y): return y + oy

    if offset_pts and len(offset_pts) >= 2:
        start = offset_pts[0]
        if lead_in_length > 0:
            cx = sum(p[0] for p in offset_pts) / len(offset_pts)
            cy = sum(p[1] for p in offset_pts) / len(offset_pts)
            if is_inner:
                # 穴(内側): 重心→始点 方向(外向き) → lead_start が穴の内側(捨て材) ✓
                dx = start[0] - cx
                dy = start[1] - cy
            else:
                # 外周(外側): 始点→重心 方向(内向き) → lead_start が輪郭の外側(捨て材) ✓
                dx = cx - start[0]
                dy = cy - start[1]
            d  = math.hypot(dx, dy)
            ld = (dx / d, dy / d) if d > 1e-10 else (1.0, 0.0)
            g0 = (_px(start[0] - ld[0] * lead_in_length),
                  _py(start[1] - ld[1] * lead_in_length))
        else:
            g0 = (_px(start[0]), _py(start[1]))

        if lead_out_length > 0 and len(offset_pts) >= 2:
            dx = offset_pts[-1][0] - offset_pts[-2][0]
            dy = offset_pts[-1][1] - offset_pts[-2][1]
            d  = math.hypot(dx, dy)
            ld = (dx / d, dy / d) if d > 1e-10 else (1.0, 0.0)
            end = (_px(offset_pts[-1][0] + ld[0] * lead_out_length),
                   _py(offset_pts[-1][1] + ld[1] * lead_out_length))
        else:
            end = (_px(offset_pts[-1][0]), _py(offset_pts[-1][1]))
    else:
        start = path.segments[0].start
        if lead_in_length > 0:
            if path.closed:
                if is_inner:
                    ld = _centroid_dir(path)          # 穴: 外側から内側へ → lead_start 内側 ✓
                else:
                    ld = _centroid_dir_outward(path)  # 外周: 内側から外側へ → lead_start 外側 ✓
            else:
                ld = _seg_direction_at_start(path.segments[0])
            g0 = (_px(start[0] - ld[0] * lead_in_length),
                  _py(start[1] - ld[1] * lead_in_length))
        else:
            g0 = (_px(start[0]), _py(start[1]))

        last_end = path.segments[-1].end
        if lead_out_length > 0:
            ld  = _seg_direction_at_end(path.segments[-1])
            end = (_px(last_end[0] + ld[0] * lead_out_length),
                   _py(last_end[1] + ld[1] * lead_out_length))
        else:
            end = (_px(last_end[0]), _py(last_end[1]))

    return g0, end


def _generate_offset_path(lines, offset_pts, fr, pierce_delay,
                           lead_in_length, lead_out_length, px, py,
                           is_inner=False):
    start = offset_pts[0]

    if lead_in_length > 0:
        cx = sum(p[0] for p in offset_pts) / len(offset_pts)
        cy = sum(p[1] for p in offset_pts) / len(offset_pts)
        if is_inner:
            # 穴: 重心→始点(外向き) → lead_start が穴の内側(捨て材) ✓
            dx = start[0] - cx
            dy = start[1] - cy
        else:
            # 外周: 始点→重心(内向き) → lead_start が輪郭の外側(捨て材) ✓
            dx = cx - start[0]
            dy = cy - start[1]
        d = math.hypot(dx, dy)
        lead_dir = (dx / d, dy / d) if d > 1e-10 else (1.0, 0.0)
        lead_start = (start[0] - lead_dir[0] * lead_in_length,
                      start[1] - lead_dir[1] * lead_in_length)
        lines.append(f'G0 X{px(lead_start[0]):.3f} Y{py(lead_start[1]):.3f}')
        lines.append('M3 ; torch on')
        lines.append(f'G4 P{pierce_delay:.2f} ; pierce delay')
        lines.append(f'G1 X{px(start[0]):.3f} Y{py(start[1]):.3f} F{fr} ; lead-in')
    else:
        lines.append(f'G0 X{px(start[0]):.3f} Y{py(start[1]):.3f}')
        lines.append('M3 ; torch on')
        lines.append(f'G4 P{pierce_delay:.2f} ; pierce delay')

    for pt in offset_pts[1:]:
        lines.append(f'G1 X{px(pt[0]):.3f} Y{py(pt[1]):.3f} F{fr}')

    if lead_out_length > 0 and len(offset_pts) >= 2:
        dx = offset_pts[-1][0] - offset_pts[-2][0]
        dy = offset_pts[-1][1] - offset_pts[-2][1]
        d = math.hypot(dx, dy)
        lo_dir = (dx / d, dy / d) if d > 1e-10 else (1.0, 0.0)
        lo_end = (offset_pts[-1][0] + lo_dir[0] * lead_out_length,
                  offset_pts[-1][1] + lo_dir[1] * lead_out_length)
        lines.append(f'G1 X{px(lo_end[0]):.3f} Y{py(lo_end[1]):.3f} F{fr} ; lead-out')


def _generate_original_path(lines, path, fr, pierce_delay,
                              lead_in_length, lead_out_length, px, py,
                              is_inner=False):
    start = path.segments[0].start

    if lead_in_length > 0:
        if path.closed:
            if is_inner:
                # 穴: 重心→始点(外向き) → lead_start が穴の内側(捨て材) ✓
                lead_dir = _centroid_dir(path)
            else:
                # 外周: 始点→重心(内向き) → lead_start が輪郭の外側(捨て材) ✓
                lead_dir = _centroid_dir_outward(path)
        else:
            lead_dir = _seg_direction_at_start(path.segments[0])
        lead_start = (
            start[0] - lead_dir[0] * lead_in_length,
            start[1] - lead_dir[1] * lead_in_length,
        )
        lines.append(f'G0 X{px(lead_start[0]):.3f} Y{py(lead_start[1]):.3f}')
        lines.append('M3 ; torch on')
        lines.append(f'G4 P{pierce_delay:.2f} ; pierce delay')
        lines.append(f'G1 X{px(start[0]):.3f} Y{py(start[1]):.3f} F{fr} ; lead-in')
    else:
        lines.append(f'G0 X{px(start[0]):.3f} Y{py(start[1]):.3f}')
        lines.append('M3 ; torch on')
        lines.append(f'G4 P{pierce_delay:.2f} ; pierce delay')

    for seg in path.segments:
        if seg.type == 'line':
            lines.append(
                f'G1 X{px(seg.end[0]):.3f} Y{py(seg.end[1]):.3f} F{fr}'
            )
        elif seg.type == 'arc':
            cx, cy, r, sa, ea, ccw = seg.data
            ix = cx - seg.start[0]
            iy = cy - seg.start[1]
            cmd = 'G3' if ccw else 'G2'
            lines.append(
                f'{cmd} X{px(seg.end[0]):.3f} Y{py(seg.end[1]):.3f} '
                f'I{ix:.3f} J{iy:.3f} F{fr}'
            )

    if lead_out_length > 0:
        last_seg = path.segments[-1]
        lo_dir = _seg_direction_at_end(last_seg)
        lo_end = (
            last_seg.end[0] + lo_dir[0] * lead_out_length,
            last_seg.end[1] + lo_dir[1] * lead_out_length,
        )
        lines.append(f'G1 X{px(lo_end[0]):.3f} Y{py(lo_end[1]):.3f} F{fr} ; lead-out')


def _centroid_dir_outward(path) -> tuple:
    """外側パス用: 始点→重心 の方向（内向き）→ lead_start が材料の外側に出る。"""
    pts = path.get_display_points()
    start = path.segments[0].start
    if not pts or len(pts) < 2:
        return _seg_direction_at_start(path.segments[0])
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    dx = cx - start[0]   # 始点→重心（内向き）
    dy = cy - start[1]
    d = math.hypot(dx, dy)
    if d < 1e-10:
        return _seg_direction_at_start(path.segments[0])
    return (dx / d, dy / d)


def _centroid_dir(path) -> tuple:
    """内側パス用: 重心→始点 の方向を返す（リードイン始点が穴の内側に入る）。"""
    pts = path.get_display_points()
    start = path.segments[0].start
    if not pts or len(pts) < 2:
        return _seg_direction_at_start(path.segments[0])
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    dx = start[0] - cx
    dy = start[1] - cy
    d = math.hypot(dx, dy)
    if d < 1e-10:
        return _seg_direction_at_start(path.segments[0])
    return (dx / d, dy / d)


def _seg_direction_at_start(seg) -> tuple:
    """Tangent direction at the START of a segment (normalized)."""
    if seg.type == 'line':
        dx = seg.end[0] - seg.start[0]
        dy = seg.end[1] - seg.start[1]
    else:
        cx, cy, r, sa, ea, ccw = seg.data
        sa_r = math.radians(sa)
        dx = -math.sin(sa_r) if ccw else math.sin(sa_r)
        dy =  math.cos(sa_r) if ccw else -math.cos(sa_r)
    d = math.hypot(dx, dy)
    return (dx / d, dy / d) if d > 1e-10 else (1.0, 0.0)


def _seg_direction_at_end(seg) -> tuple:
    """Tangent direction at the END of a segment (normalized)."""
    if seg.type == 'line':
        dx = seg.end[0] - seg.start[0]
        dy = seg.end[1] - seg.start[1]
    else:
        cx, cy, r, sa, ea, ccw = seg.data
        ea_r = math.radians(ea)
        dx = -math.sin(ea_r) if ccw else math.sin(ea_r)
        dy =  math.cos(ea_r) if ccw else -math.cos(ea_r)
    d = math.hypot(dx, dy)
    return (dx / d, dy / d) if d > 1e-10 else (1.0, 0.0)
