#!/usr/bin/env python3
"""Lighter meshes for RViz / MoveIt / Gazebo (docs/decisions/0026). NOT USED YET: first the full meshes are tried.

    pip install trimesh fast-simplification rtree scipy networkx
    python3 scripts/simplify_meshes.py <folder of Fusion's STL> [max faces per mesh, default 20000]

Fusion exports at full resolution (one link of 77 MB, 114 MB in all): too big for git, and slow for collision
checks. A link's STL holds many bodies (servos, gears, screws: 372 in the gripper base), so it is reduced body by
body: each body gets a share of the faces in proportion to its surface, at least MIN_FACES, and is reduced with
quadric decimation (shape kept, flat areas lose triangles); modelled screw threads, which decimation cannot
reduce, become their convex hull (a cylinder).
Every body stays within TOLERANCE_MM of the original. The result goes to meshes/ of this package; the
largest deviation from the original is printed (mm, the units of the STL), so a reduction that changes the shape
is visible.
"""

import sys
from pathlib import Path

import fast_simplification
import numpy as np
import trimesh

TARGET = Path(__file__).resolve().parents[1] / "meshes"
MIN_FACES = 60          # a screw keeps a recognisable shape
TOLERANCE_MM = 1.0      # no body may move further than this from the original surface


def deviation_mm(original, reduced, samples=20000) -> float:
    """Largest distance from points on the original to the reduced surface (a one-sided Hausdorff estimate)."""
    points, _ = trimesh.sample.sample_surface(original, samples, seed=1)
    _, distance, _ = trimesh.proximity.closest_point(reduced, points)
    return float(np.max(distance))


def decimate(body, budget: int):
    if len(body.faces) <= budget:
        return body
    vertices, triangles = fast_simplification.simplify(body.vertices, body.faces, 1.0 - budget / len(body.faces))
    return trimesh.Trimesh(vertices, triangles) if len(triangles) else body


def reduce_body(body, budget: int):
    """Decimation within TOLERANCE_MM: the face budget doubles until the body is close enough (thin plates and
    rings collapse when reduced too far). A body that refuses to shrink (modelled screw threads stall at ~30 %
    of their faces) becomes its convex hull, a cylinder for a screw: the error is the depth of the thread."""
    while budget < len(body.faces):
        reduced = decimate(body, budget)
        if len(reduced.faces) > 2 * budget:
            hull = body.convex_hull
            if deviation_mm(body, hull, 2000) <= TOLERANCE_MM:
                return decimate(hull, budget) if len(hull.faces) > budget else hull
        elif deviation_mm(body, reduced, 2000) <= TOLERANCE_MM:
            return reduced
        budget *= 2
    return body


def simplify(source: Path, max_faces: int) -> tuple:
    mesh = trimesh.load_mesh(source)
    faces = len(mesh.faces)
    if faces > max_faces:
        bodies = mesh.split(only_watertight=False, repair=False)
        area = sum(b.area for b in bodies) or 1.0
        reduced = trimesh.util.concatenate([reduce_body(b, max(MIN_FACES, int(max_faces * b.area / area)))
                                            for b in bodies])
        error = deviation_mm(mesh, reduced)
    else:
        reduced, error = mesh, 0.0
    reduced.export(TARGET / source.name)
    return faces, len(reduced.faces), error


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if not argv:
        raise SystemExit(__doc__)
    max_faces = int(argv[1]) if len(argv) > 1 else 20000
    TARGET.mkdir(exist_ok=True)
    for source in sorted(Path(argv[0]).glob("*.stl")):
        before, after, error = simplify(source, max_faces)
        size = (TARGET / source.name).stat().st_size / 1e6
        print(f"{source.name:26s} {before:8d} → {after:6d} faces  {size:6.2f} MB  max deviation {error:.2f} mm")


if __name__ == "__main__":
    main()
