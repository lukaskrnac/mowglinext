#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: GPL-3.0-or-later
"""
align_glim_map.py — one-time offline alignment of a GLIM .ply map into the
SAME ENU frame fusion_graph uses (map frame: x=east, y=north, metres,
origin at datum_lat/datum_lon — see fusion_graph_node_timer.cpp's
LatLonToMap()).

Why this is needed:
    GLIM builds its map in its own arbitrary local frame (wherever mapping
    happened to start, at whatever heading the robot faced). fusion_graph_node
    reads /pcl_pose in its own `map` frame and does NOT re-project it — see
    OnLidarPose's frame_id check in fusion_graph_node_lidar_primary.cpp. If
    you start every mapping run parked at the dock, TRANSLATION is already
    solved (both frames agree the dock is near the origin) — but ROTATION
    (which way GLIM's local +x/+y point vs true east/north) is NOT solved
    just by that, and must be fixed here, once, offline.

What this script does:
    1. Loads the raw GLIM .ply.
    2. Lets you interactively pick >=2 point pairs: click a point in the
       Open3D viewer, then enter its known ENU (east, north) coordinate in
       fusion_graph's map frame (e.g. from an RTK-Fixed /gps/absolute_pose
       reading taken at that physical spot during/after the mapping run).
       The dock itself (0, 0) is always a trivially available pair if you
       started mapping parked at the dock.
    3. Computes the best-fit 2D similarity transform (rotation + optional
       scale + translation) with the Umeyama method and reports the residual
       per point (this is your alignment error budget — a few cm is
       reasonable with good GPS reference points, much more means a bad
       reference pick, not a bad map).
    4. Applies the transform to the full point cloud and writes it back out
       as a .pcd (lidar_localization_ros2 accepts .pcd or .ply directly —
       .pcd is used here because it's the more commonly tested path in their
       docs).

Usage:
    python3 align_glim_map.py raw_map.ply aligned_map.pcd

Dependencies: open3d, numpy. `pip install --break-system-packages open3d numpy`
if not already present in your workspace.

This is a starting point, not a finished tool — in particular:
  - Scale should come out very close to 1.0 (GLIM's LiDAR odometry is
    already metric); a scale far from 1.0 is a sign of a bad reference pick,
    not something to silently trust and apply.
  - For higher precision than manual point-picking gives you, align full
    trajectories instead (GLIM's own pose log vs an RTK-Fixed GPS log from
    the same run) — a future upgrade, not implemented here.
"""

import sys

import numpy as np
import open3d as o3d


def umeyama_2d(src_xy: np.ndarray, dst_xy: np.ndarray, estimate_scale: bool = True):
    """Best-fit similarity transform mapping src_xy -> dst_xy (both Nx2).

    Returns (R (2x2), t (2,), scale (float)). Standard Umeyama (1991) least-
    squares closed form; N>=2 required, N>=3 recommended for a redundant,
    checkable fit (2 points alone cannot detect a bad third-axis / mirrored
    pick).
    """
    assert src_xy.shape == dst_xy.shape and src_xy.shape[0] >= 2
    mu_src = src_xy.mean(axis=0)
    mu_dst = dst_xy.mean(axis=0)
    src_c = src_xy - mu_src
    dst_c = dst_xy - mu_dst
    cov = (dst_c.T @ src_c) / src_xy.shape[0]
    u, d, vt = np.linalg.svd(cov)
    s = np.eye(2)
    if np.linalg.det(u) * np.linalg.det(vt) < 0.0:
        s[-1, -1] = -1.0
    r = u @ s @ vt
    if estimate_scale:
        var_src = (src_c**2).sum() / src_xy.shape[0]
        scale = np.trace(np.diag(d) @ s) / var_src if var_src > 1e-12 else 1.0
    else:
        scale = 1.0
    t = mu_dst - scale * r @ mu_src
    return r, t, scale


def pick_reference_pairs(pcd: "o3d.geometry.PointCloud"):
    print(
        "\nPick each reference point in the viewer window: shift+click a point, "
        "then close the window (or press 'q') when done picking THIS ONE point. "
        "You'll be asked for its known map-frame (east, north) coordinate "
        "immediately after. Repeat for at least 2 (ideally 3+) well-separated "
        "points — the dock (0, 0) is usually the easiest first pick.\n"
    )
    src_pts = []
    dst_pts = []
    while True:
        vis = o3d.visualization.VisualizerWithVertexSelection()
        vis.create_window(window_name=f"Pick reference point #{len(src_pts) + 1} (or close to stop)")
        vis.add_geometry(pcd)
        vis.run()
        picked = vis.get_picked_points()
        vis.destroy_window()
        if not picked:
            break
        idx = picked[0].index
        p = np.asarray(pcd.points)[idx]
        print(f"  picked point index {idx}: GLIM-frame (x={p[0]:.3f}, y={p[1]:.3f}, z={p[2]:.3f})")
        raw = input("  enter its known map-frame 'east north' in metres (e.g. '0 0' for the dock): ")
        e, n = (float(v) for v in raw.split())
        src_pts.append([p[0], p[1]])
        dst_pts.append([e, n])
        cont = input("  pick another point? [Y/n]: ").strip().lower()
        if cont == "n":
            break
    if len(src_pts) < 2:
        raise SystemExit("Need at least 2 reference pairs.")
    return np.array(src_pts), np.array(dst_pts)


def main():
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} raw_map.ply aligned_map.pcd")
        raise SystemExit(1)
    in_path, out_path = sys.argv[1], sys.argv[2]

    pcd = o3d.io.read_point_cloud(in_path)
    print(f"loaded {len(pcd.points)} points from {in_path}")

    src_xy, dst_xy = pick_reference_pairs(pcd)
    r, t, scale = umeyama_2d(src_xy, dst_xy, estimate_scale=True)

    # Residual check per point — this IS your alignment error budget.
    fitted = (scale * (r @ src_xy.T).T) + t
    residuals = np.linalg.norm(fitted - dst_xy, axis=1)
    print("\nPer-point residuals (m):")
    for i, res in enumerate(residuals):
        print(f"  point {i}: {res:.4f} m")
    print(f"\nrotation (deg): {np.degrees(np.arctan2(r[1, 0], r[0, 0])):.3f}")
    print(f"scale: {scale:.6f}  (expect close to 1.0 — investigate if not)")
    print(f"translation (e, n): ({t[0]:.3f}, {t[1]:.3f})")

    if abs(scale - 1.0) > 0.02:
        print(
            "\nWARNING: scale is more than 2% away from 1.0. GLIM's LiDAR "
            "odometry is metric already — a scale this far off almost always "
            "means a bad reference-point pick (wrong point clicked, or a "
            "typo'd coordinate), not a real map scale error. Re-check your "
            "picks before trusting this transform."
        )

    points = np.asarray(pcd.points)
    xy = points[:, :2]
    z = points[:, 2:3]
    xy_aligned = (scale * (r @ xy.T).T) + t
    aligned = np.hstack([xy_aligned, z])  # z (height) is untouched: 2D similarity only

    out = o3d.geometry.PointCloud()
    out.points = o3d.utility.Vector3dVector(aligned)
    if pcd.has_colors():
        out.colors = pcd.colors
    if pcd.has_normals():
        # Normals' xy components should rotate too; z-only 2D transform above
        # leaves them approximately valid for a mostly-planar rotation. Drop
        # them if lidar_localization_ros2's importer doesn't need them.
        out.normals = pcd.normals
    o3d.io.write_point_cloud(out_path, out)
    print(f"\nwrote {len(out.points)} aligned points to {out_path}")
    print(
        "\nNext: point lidar_localization_bringup.launch.py's map_path at "
        f"{out_path} and verify in RViz that it overlays your GPS trajectory "
        "correctly before trusting it in the field."
    )


if __name__ == "__main__":
    main()
