import math
from typing import List
from dxf_reader import Path
from kerf_offset import compute_offset_points, compute_path_types


def generate_gcode(dxf_entries: list, settings: dict) -> str:
    """
    dxf_entries: list of {'paths': [...], 'offset_x': float, 'offset_y': float}
    """
    feed_rate = settings.get('feed_rate', 3000)
    pierce_delay = settings.get('pierce_delay', 0.5)
    lead_in_length = settings.get('lead_in_length', 5.0)
    lead_in_type = settings.get('lead_in_type', 'line')
    lead_out_length = settings.get('lead_out_length', 0.0)
    kerf_width = settings.get('kerf_width', 0.0)

    fr = int(feed_rate)  # GRBLは整数送り速度を推奨

    lines = [
        '; Plasma CAM G-code',
        f'; Feed rate: {fr} mm/min',
        f'; Pierce delay: {pierce_delay} s',
        f'; Lead-in: {lead_in_length} mm ({lead_in_type})',
        f'; Lead-out: {lead_out_length} mm',
        f'; Kerf width: {kerf_width} mm',
        '',
        'G21',   # mmモード
        'G90',   # 絶対座標
        'G94',   # 毎分送り（GRBL必須）
        'M5',    # トーチ確実OFF
        'G0 X0 Y0',
        '',
    ]

    path_global_idx = 0

    for entry in dxf_entries:
        ox = entry.get('offset_x', 0.0)
        oy = entry.get('offset_y', 0.0)
        paths = entry['paths']

        def px(x): return x + ox
        def py(y): return y + oy

        # Spatial nesting detection per file
        inner_flags = compute_path_types(paths) if kerf_width > 0 else [False] * len(paths)

        for i, path in enumerate(paths):
            if not path.segments:
                path_global_idx += 1
                continue

            lines.append(f'; --- Path {path_global_idx + 1} ({entry["name"]}) ---')
            path_global_idx += 1

            # Attempt kerf offset for closed paths
            offset_pts = None
            if kerf_width > 0 and path.closed:
                offset_pts = compute_offset_points(path, kerf_width, is_inner=inner_flags[i])

            if offset_pts and len(offset_pts) >= 2:
                _generate_offset_path(lines, offset_pts, fr, pierce_delay,
                                      lead_in_length, lead_out_length, px, py)
            else:
                _generate_original_path(lines, path, fr, pierce_delay,
                                        lead_in_length, lead_out_length, px, py)

            lines.append('M5 ; torch off')
            lines.append('')

    lines += ['G0 X0 Y0', 'M30 ; program end']
    return '\n'.join(lines)


def _generate_offset_path(lines, offset_pts, fr, pierce_delay,
                           lead_in_length, lead_out_length, px, py):
    start = offset_pts[0]

    if lead_in_length > 0:
        dx = offset_pts[1][0] - offset_pts[0][0]
        dy = offset_pts[1][1] - offset_pts[0][1]
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
                              lead_in_length, lead_out_length, px, py):
    start = path.segments[0].start

    if lead_in_length > 0:
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
