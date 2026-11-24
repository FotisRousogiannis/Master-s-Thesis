#!/usr/bin/env python3
"""Light meshes for collision checks (MoveIt, Gazebo), docs/decisions/0026.

    pip install trimesh fast-simplification rtree scipy networkx
    python3 scripts/simplify_meshes.py <folder of Fusion's STL> [max faces per mesh] [tolerance mm]

The full meshes (from the export zip) are only drawn. With them for collisions, move_group needed 86 s to start on
the laptop (2026-11-24); the light ones here, in meshes_collision/ of this package, are used instead.

Fusion exports at full resolution (one link of 77 MB, 114 MB in all): too big for git, and slow for collision
checks. A link's STL holds many bodies (servos, gears, screws: 372 in the gripper base), so it is reduced body by
body (see reduce_body): its convex hull where that is close (≤ HULL_MM, always on the safe side), otherwise
quadric decimation within TOLERANCE_MM, with a share of the faces in proportion to its surface.
The result goes to meshes_collision/ of this package; the
largest deviation from the original is printed (mm, the units of the STL), so a reduction that changes the shape
is visible.
"""

import sys
from pathlib import Path

import fast_simplification
import numpy as np
import trimesh

TARGET = Path(__file__).resolve().parents[1] / "meshes_collision"
MIN_FACES = 60          # a screw keeps a recognisable shape
MAX_FACES = 8000        # per link
TOLERANCE_MM = 2.0      # no body may move further than this from the original surface (collision margin)
HULL_MM = 5.0           # a body whose convex hull bulges less than this becomes its hull


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
    """One body for collision checks. First its convex hull: it always contains the body, so the check stays safe
    (MoveIt only keeps a little more distance), and it is cheap; screws, gears, shafts are almost convex. If the
    hull bulges more than HULL_MM from the body (a bracket, a housing with pockets), quadric decimation instead,
    with the face budget doubling until the body is within TOLERANCE_MM."""
    try:
        hull = body.convex_hull
    except Exception:                       # noqa: B902 (a flat or tiny body: qhull has nothing to wrap)
        return body
    if deviation_mm(body, hull, 1000) <= HULL_MM:
        return decimate(hull, budget) if len(hull.faces) > budget else hull
    while budget < len(body.faces):
        reduced = decimate(body, budget)
        if len(reduced.faces) <= 2 * budget and deviation_mm(body, reduced, 2000) <= TOLERANCE_MM:
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
    global TOLERANCE_MM
    max_faces = int(argv[1]) if len(argv) > 1 else MAX_FACES
    TOLERANCE_MM = float(argv[2]) if len(argv) > 2 else TOLERANCE_MM
    TARGET.mkdir(exist_ok=True)
    for source in sorted(Path(argv[0]).glob("*.stl")):
        before, after, error = simplify(source, max_faces)
        size = (TARGET / source.name).stat().st_size / 1e6
        print(f"{source.name:26s} {before:8d} → {after:6d} faces  {size:6.2f} MB  max deviation {error:.2f} mm")


if __name__ == "__main__":
    main()
