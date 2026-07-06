import ezdxf
import math
from typing import List, Tuple

TOLERANCE = 0.5       # チェーン結合の許容距離 (mm)  ※Rコーナーの接続ミス防止のため0.5に設定
AUTO_CLOSE_TOL = 1.0  # この距離以内なら自動クローズ (mm)


class Segment:
    def __init__(self, seg_type, start, end, data):
        self.type = seg_type  # 'line' or 'arc'
        self.start = start    # (x, y)
        self.end = end        # (x, y)
        self.data = data


class Path:
    def __init__(self, segments: List[Segment], closed: bool = False):
        self.segments = segments
        self.closed = closed
        self._cache = {}  # キャッシュ: {resolution: points}

    def get_display_points(self, resolution=36):
        if resolution in self._cache:
            return self._cache[resolution]
        points = []
        for seg in self.segments:
            if seg.type == 'line':
                if not points:
                    points.append(seg.start)
                points.append(seg.end)
            elif seg.type == 'arc':
                pts = arc_to_points(seg.data, resolution)
                if points:
                    pts = pts[1:]
                points.extend(pts)
        if self.closed and len(points) > 1:
            points.append(points[0])
        self._cache[resolution] = points
        return points


def arc_to_points(data, resolution=36):
    cx, cy, r, sa, ea, ccw = data
    start_r = math.radians(sa)
    end_r = math.radians(ea)
    if ccw:
        if end_r <= start_r:
            end_r += 2 * math.pi
    else:
        if end_r >= start_r:
            end_r -= 2 * math.pi
    return [
        (cx + r * math.cos(start_r + (end_r - start_r) * i / resolution),
         cy + r * math.sin(start_r + (end_r - start_r) * i / resolution))
        for i in range(resolution + 1)
    ]


def points_equal(p1, p2, tol=TOLERANCE):
    return abs(p1[0] - p2[0]) < tol and abs(p1[1] - p2[1]) < tol


def points_dist(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


def entity_to_segments(entity) -> List[Segment]:
    segs = []
    t = entity.dxftype()

    if t == 'LINE':
        s = (entity.dxf.start.x, entity.dxf.start.y)
        e = (entity.dxf.end.x, entity.dxf.end.y)
        segs.append(Segment('line', s, e, (s, e)))

    elif t == 'ARC':
        cx = entity.dxf.center.x
        cy = entity.dxf.center.y
        r = entity.dxf.radius
        sa = entity.dxf.start_angle
        ea = entity.dxf.end_angle
        s = (cx + r * math.cos(math.radians(sa)),
             cy + r * math.sin(math.radians(sa)))
        e = (cx + r * math.cos(math.radians(ea)),
             cy + r * math.sin(math.radians(ea)))
        segs.append(Segment('arc', s, e, (cx, cy, r, sa, ea, True)))

    elif t == 'CIRCLE':
        cx = entity.dxf.center.x
        cy = entity.dxf.center.y
        r = entity.dxf.radius
        # GRBLは start=end の全円 G2/G3 を実行しない場合があるため
        # 180°ずつ2本のアークに分割する
        s1 = (cx + r, cy)   # 0°
        s2 = (cx - r, cy)   # 180°
        segs.append(Segment('arc', s1, s2, (cx, cy, r,   0, 180, True)))
        segs.append(Segment('arc', s2, s1, (cx, cy, r, 180, 360, True)))

    elif t == 'LWPOLYLINE':
        pts = list(entity.get_points())
        closed = entity.closed
        n = len(pts)
        indices = list(range(n))
        if closed:
            indices.append(0)
        for i in range(len(indices) - 1):
            i1, i2 = indices[i], indices[i + 1]
            p1 = (pts[i1][0], pts[i1][1])
            p2 = (pts[i2][0], pts[i2][1])
            bulge = pts[i1][4] if len(pts[i1]) > 4 else 0
            if abs(bulge) < 1e-10:
                segs.append(Segment('line', p1, p2, (p1, p2)))
            else:
                arc_seg = bulge_to_arc(p1, p2, bulge)
                if arc_seg:
                    segs.append(arc_seg)

    elif t == 'SPLINE':
        try:
            pts = list(entity.flattening(0.1))
            for i in range(len(pts) - 1):
                p1 = (pts[i].x, pts[i].y)
                p2 = (pts[i + 1].x, pts[i + 1].y)
                segs.append(Segment('line', p1, p2, (p1, p2)))
        except Exception:
            pass

    elif t == 'POLYLINE':
        # 旧式 2D ポリライン
        try:
            pts = [(v.dxf.location.x, v.dxf.location.y) for v in entity.vertices]
            closed = bool(entity.dxf.flags & 1)
            n = len(pts)
            if n >= 2:
                for i in range(n - 1 + (1 if closed else 0)):
                    p1 = pts[i % n]
                    p2 = pts[(i + 1) % n]
                    segs.append(Segment('line', p1, p2, (p1, p2)))
        except Exception:
            pass

    elif t == 'ELLIPSE':
        try:
            pts = list(entity.flattening(0.1))
            for i in range(len(pts) - 1):
                p1 = (pts[i].x, pts[i].y)
                p2 = (pts[i + 1].x, pts[i + 1].y)
                segs.append(Segment('line', p1, p2, (p1, p2)))
        except Exception:
            pass

    return segs


def bulge_to_arc(p1, p2, bulge) -> Segment:
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    d = math.sqrt(dx * dx + dy * dy)
    if d < 1e-10:
        return None

    r = d * (1 + bulge * bulge) / (4 * abs(bulge))
    dist_to_center = math.sqrt(max(0, r * r - (d / 2) ** 2))

    mx = (p1[0] + p2[0]) / 2
    my = (p1[1] + p2[1]) / 2
    px = -dy / d
    py = dx / d

    if bulge > 0:
        cx = mx + dist_to_center * px
        cy = my + dist_to_center * py
    else:
        cx = mx - dist_to_center * px
        cy = my - dist_to_center * py

    sa = math.degrees(math.atan2(p1[1] - cy, p1[0] - cx))
    ea = math.degrees(math.atan2(p2[1] - cy, p2[0] - cx))
    ccw = bulge > 0

    return Segment('arc', p1, p2, (cx, cy, r, sa, ea, ccw))


def reverse_segment(seg: Segment) -> Segment:
    if seg.type == 'line':
        return Segment('line', seg.end, seg.start, (seg.end, seg.start))
    elif seg.type == 'arc':
        cx, cy, r, sa, ea, ccw = seg.data
        return Segment('arc', seg.end, seg.start, (cx, cy, r, ea, sa, not ccw))
    return seg


def chain_segments(segments: List[Segment]) -> List[Path]:
    if not segments:
        return []

    used = [False] * len(segments)
    paths = []

    while True:
        start_idx = next((i for i, u in enumerate(used) if not u), None)
        if start_idx is None:
            break

        chain = [segments[start_idx]]
        used[start_idx] = True

        if points_equal(chain[0].start, chain[0].end):
            paths.append(Path(chain, closed=True))
            continue

        closed = False
        while True:
            extended = False

            # 末尾方向へ伸ばす
            current_end = chain[-1].end
            for i, seg in enumerate(segments):
                if used[i]:
                    continue
                if points_equal(seg.start, current_end):
                    chain.append(seg)
                    used[i] = True
                    extended = True
                    break
                elif points_equal(seg.end, current_end):
                    chain.append(reverse_segment(seg))
                    used[i] = True
                    extended = True
                    break

            # 先頭方向へ伸ばす
            if not extended:
                current_start = chain[0].start
                for i, seg in enumerate(segments):
                    if used[i]:
                        continue
                    if points_equal(seg.end, current_start):
                        chain.insert(0, seg)
                        used[i] = True
                        extended = True
                        break
                    elif points_equal(seg.start, current_start):
                        chain.insert(0, reverse_segment(seg))
                        used[i] = True
                        extended = True
                        break

            if not extended:
                break

            if points_equal(chain[-1].end, chain[0].start):
                closed = True
                break

        # 始点と終点が AUTO_CLOSE_TOL 以内なら自動クローズ
        if not closed and len(chain) >= 2:
            dist = points_dist(chain[0].start, chain[-1].end)
            if dist <= AUTO_CLOSE_TOL:
                closed = True

        paths.append(Path(chain, closed=closed))

    return paths


def collect_segments(entity, segments: list):
    """INSERT（ブロック参照）を再帰展開しながらセグメントを収集する"""
    if entity.dxftype() == 'INSERT':
        try:
            for sub in entity.virtual_entities():
                collect_segments(sub, segments)
        except Exception:
            pass
    else:
        segments.extend(entity_to_segments(entity))


def read_dxf(filename: str) -> List[Path]:
    doc = ezdxf.readfile(filename)
    msp = doc.modelspace()
    segments = []
    entity_types: dict = {}
    for entity in msp:
        t = entity.dxftype()
        entity_types[t] = entity_types.get(t, 0) + 1
        collect_segments(entity, segments)
    paths = chain_segments(segments)
    # 診断情報を属性として付与（main.py でログ出力に使う）
    read_dxf._last_entity_types = entity_types
    return paths
