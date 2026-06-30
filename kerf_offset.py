"""
Kerf width compensation using polygon offsetting (Shapely).

outer contour (not nested inside another path): torch travels OUTSIDE → expand by kerf/2
inner contour / hole (nested inside another path): torch travels INSIDE → shrink by kerf/2

Nesting is determined spatially (centroid containment) rather than by winding direction,
because DXF files do not guarantee a consistent winding convention.
"""
from shapely.geometry import Polygon
from typing import List, Optional, Tuple


def _build_poly(path, resolution: int = 128) -> Optional[Polygon]:
    pts = path.get_display_points(resolution=resolution)
    if len(pts) < 3:
        return None
    if pts[0] != pts[-1]:
        pts = list(pts) + [pts[0]]
    try:
        poly = Polygon(pts)
    except Exception:
        return None
    if not poly.is_valid:
        poly = poly.buffer(0)
    if poly.is_empty or poly.area < 1e-6:
        return None
    return poly


def compute_path_types(paths) -> List[bool]:
    """
    Determine inner/outer for each path using spatial nesting.

    Returns a list of booleans: True = inner (hole), False = outer.

    Detection method:
    1. Shapely full-polygon containment (most accurate; robust to concentric
       shapes sharing the same centroid, unlike a centroid-point test)
    2. Bounding-box containment fallback (handles open/invalid outer boundary)
    """
    polys = [_build_poly(p, resolution=64) if p.closed else None for p in paths]

    # バウンディングボックスを全パスで計算（open pathも含む）
    bboxes = []
    for path in paths:
        pts = path.get_display_points()
        if pts:
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            bboxes.append((min(xs), min(ys), max(xs), max(ys)))
        else:
            bboxes.append(None)

    result = []
    for i, poly in enumerate(polys):
        if poly is None:
            result.append(False)
            continue

        # ── 方法1: ポリゴン全体の内包チェック ──────────────
        # 重心点だけで判定すると、同心円のように複数パスの重心が
        # 一致するケースで大小関係を区別できず深さが壊れるため、
        # ポリゴン全体が完全に包含されているかで判定する。
        depth = sum(
            1 for j, other in enumerate(polys)
            if j != i and other is not None and other.contains(poly)
        )

        # ── 方法2: バウンディングボックス内包フォールバック ───
        # 外枠が閉じていない・ポリゴン構築失敗の場合に使用
        if depth == 0 and bboxes[i] is not None:
            xi1, yi1, xi2, yi2 = bboxes[i]
            for j, bb in enumerate(bboxes):
                if j == i or bb is None:
                    continue
                # path j の bbox が path i の bbox を完全に包んでいるか
                if bb[0] < xi1 and bb[1] < yi1 and bb[2] > xi2 and bb[3] > yi2:
                    depth += 1

        result.append(depth % 2 == 1)
    return result


def compute_offset_points(
    path,
    kerf_width: float,
    is_inner: Optional[bool] = None,
    resolution: int = 128,
) -> Optional[List[Tuple[float, float]]]:
    """
    Compute kerf-compensated toolpath.

    is_inner: True = hole (shrink), False = outer (expand).
              If None, falls back to winding-direction detection.

    Returns list of (x, y) tuples for the offset path, or None if not
    applicable (open path, zero kerf, or degenerate geometry).
    """
    if not path.closed or kerf_width <= 0:
        return None

    poly = _build_poly(path, resolution=resolution)
    if poly is None:
        return None

    if is_inner is None:
        pts = path.get_display_points()
        is_inner = signed_area(pts) > 0

    offset_dist = -(kerf_width / 2) if is_inner else (kerf_width / 2)

    try:
        offset_poly = poly.buffer(
            offset_dist,
            resolution=resolution,
            join_style=1,       # round joins (アーク形状を保持)
        )
    except Exception:
        return None

    if offset_poly.is_empty:
        return None

    if offset_poly.geom_type == 'MultiPolygon':
        offset_poly = max(offset_poly.geoms, key=lambda g: g.area)

    if offset_poly.geom_type != 'Polygon':
        return None

    return list(offset_poly.exterior.coords)


def signed_area(pts: List[Tuple[float, float]]) -> float:
    """Shoelace formula. Positive = CCW, Negative = CW."""
    n = len(pts)
    if n < 3:
        return 0.0
    return sum(
        pts[i][0] * pts[(i + 1) % n][1] - pts[(i + 1) % n][0] * pts[i][1]
        for i in range(n)
    ) / 2.0


def is_inner_contour(pts: List[Tuple[float, float]]) -> bool:
    """Winding-direction fallback (CCW = hole). Prefer compute_path_types() when possible."""
    return signed_area(pts) > 0


def contour_label(pts: List[Tuple[float, float]], closed: bool) -> str:
    if not closed:
        return '開路'
    return '内側(穴)' if is_inner_contour(pts) else '外側'
