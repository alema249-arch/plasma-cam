"""
Kerf width compensation using polygon offsetting (Shapely).

outer contour (not nested inside another path): torch travels OUTSIDE → expand by kerf/2
inner contour / hole (nested inside another path): torch travels INSIDE → shrink by kerf/2

Nesting is determined spatially (centroid containment) rather than by winding direction,
because DXF files do not guarantee a consistent winding convention.
"""
from shapely.geometry import Polygon
from typing import List, Optional, Tuple


def _build_poly(path) -> Optional[Polygon]:
    pts = path.get_display_points()
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
    A path whose centroid lies inside another closed path is a hole.
    """
    polys = [_build_poly(p) if p.closed else None for p in paths]

    result = []
    for i, poly in enumerate(polys):
        if poly is None:
            result.append(False)
            continue
        centroid = poly.centroid
        # Count how many other closed polygons contain this centroid
        depth = sum(
            1 for j, other in enumerate(polys)
            if j != i and other is not None and other.contains(centroid)
        )
        # Odd nesting depth = hole, even (including 0) = outer
        result.append(depth % 2 == 1)
    return result


def compute_offset_points(
    path,
    kerf_width: float,
    is_inner: Optional[bool] = None,
    resolution: int = 32,
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

    poly = _build_poly(path)
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
            join_style=2,       # mitered corners
            mitre_limit=5.0,
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
