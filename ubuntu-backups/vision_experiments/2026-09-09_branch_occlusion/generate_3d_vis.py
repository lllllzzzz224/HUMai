#!/usr/bin/env python3
"""Generate 3D point cloud & sphere fit visualization."""

from __future__ import annotations

import json
from pathlib import Path
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

DATA_DIR = Path("/home/li/vision_experiments/2026-09-09_branch_occlusion/data")
OUT_DIR = Path("/home/li/vision_experiments/2026-09-09_branch_occlusion")

def main():
    color = cv2.imread(str(DATA_DIR / "branch_color.jpg"))
    depth = cv2.imread(str(DATA_DIR / "branch_depth.png"), cv2.IMREAD_UNCHANGED)
    with open(DATA_DIR / "camera_info.json") as f:
        ci = json.load(f)

    fx = ci['k'][0]
    fy = ci['k'][4]
    cx = ci['k'][2]
    cy = ci['k'][5]

    # Apple #2 (on the vine): around row 320:410, col 275:365
    hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, np.array([0, 70, 70]), np.array([15, 255, 255]))
    m2 = cv2.inRange(hsv, np.array([160, 70, 70]), np.array([180, 255, 255]))
    rm = m1 | m2

    mask = np.zeros(depth.shape[:2], dtype=bool)
    mask[320:410, 275:365] = rm[320:410, 275:365].astype(bool)

    ys, xs = np.nonzero(mask)
    d_vals = depth[ys, xs].astype(np.float64) * 0.001
    valid = (d_vals > 0.3) & (d_vals < 0.8)
    xs, ys, d_vals = xs[valid], ys[valid], d_vals[valid]

    # Get RGB colors from original image
    b_vals = color[ys, xs, 0] / 255.0
    g_vals = color[ys, xs, 1] / 255.0
    r_vals = color[ys, xs, 2] / 255.0
    colors_rgb = np.column_stack([r_vals, g_vals, b_vals])

    X = (xs - cx) * d_vals / fx * 1000.0
    Y = (ys - cy) * d_vals / fy * 1000.0
    Z = d_vals * 1000.0
    pts = np.column_stack([X, Y, Z])

    # Sphere parameters from previous fit
    # Center = [4.0, 108.3, 489.3] mm, R = 31.0 mm
    from analyze_branches import estimate_method_b
    fit = estimate_method_b(mask, depth, fx, fy, cx, cy)
    center = np.array(fit['center']) * 1000.0  # mm
    radius = float(fit['radius_mm'])          # mm
    rmse = float(fit['rmse_mm'])

    # Distances to sphere center
    dists = np.linalg.norm(pts - center, axis=1)
    resids = np.abs(dists - radius)
    inliers = resids < 4.0

    # 1. Matplotlib Plot
    fig = plt.figure(figsize=(16, 7), dpi=150)
    fig.patch.set_facecolor('#1a1a1a')

    # Subplot 1: 3D view
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    ax1.set_facecolor('#1a1a1a')
    ax1.xaxis.pane.set_facecolor('#222222')
    ax1.yaxis.pane.set_facecolor('#222222')
    ax1.zaxis.pane.set_facecolor('#222222')
    ax1.tick_params(colors='white')
    ax1.xaxis.label.set_color('white')
    ax1.yaxis.label.set_color('white')
    ax1.zaxis.label.set_color('white')

    # Downsample points for clear visualization
    stride = max(1, len(pts) // 800)
    p_sub = pts[::stride]
    c_sub = colors_rgb[::stride]

    ax1.scatter(p_sub[:, 0], p_sub[:, 1], p_sub[:, 2], c=c_sub, s=12, alpha=0.85, edgecolors='none', label='Apple Surface Point Cloud')

    # Generate 3D sphere wireframe
    u = np.linspace(0, 2 * np.pi, 28)
    v = np.linspace(0, np.pi, 28)
    xs_sp = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    ys_sp = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    zs_sp = center[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))

    ax1.plot_wireframe(xs_sp, ys_sp, zs_sp, color='#00e5ff', alpha=0.25, linewidth=0.7, label=f'Fitted Sphere (R={radius:.1f}mm)')
    ax1.scatter([center[0]], [center[1]], [center[2]], color='#ff1744', s=100, marker='X', label=f'Sphere Center ({center[0]:.1f}, {center[1]:.1f}, {center[2]:.1f})')

    ax1.set_xlabel('X (mm)')
    ax1.set_ylabel('Y (mm)')
    ax1.set_zlabel('Depth Z (mm)')
    ax1.set_title(f'3D Point Cloud & Sphere Fit (RMSE={rmse:.2f}mm)', color='white', fontsize=13, pad=10)
    ax1.legend(loc='upper right', facecolor='#2a2a2a', edgecolor='#555555', labelcolor='white', fontsize=8)

    # Subplot 2: Cross Section (X vs Depth Z)
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.set_facecolor('#222222')
    ax2.tick_params(colors='white')
    ax2.xaxis.label.set_color('white')
    ax2.yaxis.label.set_color('white')
    for spine in ax2.spines.values():
        spine.set_color('#444444')

    ax2.scatter(pts[::stride, 0], pts[::stride, 2], c=colors_rgb[::stride], s=8, alpha=0.8)
    theta = np.linspace(0, 2 * np.pi, 200)
    ax2.plot(center[0] + radius * np.cos(theta), center[2] + radius * np.sin(theta), color='#00e5ff', linewidth=1.8, label=f'Sphere Contour (R={radius:.1f}mm)')
    ax2.plot(center[0], center[2], 'rX', markersize=10, label='Fitted Center')
    ax2.set_xlabel('X (mm)')
    ax2.set_ylabel('Z / Depth (mm)')
    ax2.set_title('Top-Down Cross Section (X vs Depth Z)', color='white', fontsize=11)
    ax2.legend(facecolor='#2a2a2a', edgecolor='#555555', labelcolor='white', fontsize=8)
    ax2.axis('equal')
    ax2.grid(True, color='#333333', linestyle='--')

    # Subplot 3: Front View (X vs Y)
    ax3 = fig.add_subplot(2, 2, 4)
    ax3.set_facecolor('#222222')
    ax3.tick_params(colors='white')
    ax3.xaxis.label.set_color('white')
    ax3.yaxis.label.set_color('white')
    for spine in ax3.spines.values():
        spine.set_color('#444444')

    ax3.scatter(pts[::stride, 0], pts[::stride, 1], c=colors_rgb[::stride], s=8, alpha=0.8)
    ax3.plot(center[0] + radius * np.cos(theta), center[1] + radius * np.sin(theta), color='#00e5ff', linewidth=1.8)
    ax3.plot(center[0], center[1], 'rX', markersize=10)
    ax3.set_xlabel('X (mm)')
    ax3.set_ylabel('Y (mm)')
    ax3.set_title('Front View Projection (X vs Y)', color='white', fontsize=11)
    ax3.axis('equal')
    ax3.invert_yaxis()
    ax3.grid(True, color='#333333', linestyle='--')

    plt.tight_layout()
    plot_file = OUT_DIR / "point_cloud_sphere_fit.png"
    plt.savefig(str(plot_file), facecolor=fig.get_facecolor(), edgecolor='none')
    plt.close()
    print(f"Plot saved: {plot_file}")

    # 2. Export JSON for interactive HTML viewer
    # Center relative to apple center so rotating around origin is intuitive
    pts_rel = pts[::stride] - center
    export_data = {
        "center_world": [round(float(v), 2) for v in center],
        "radius_mm": round(radius, 2),
        "rmse_mm": round(rmse, 2),
        "total_points": len(pts),
        "inliers_count": fit['inliers'],
        "points": [
            {
                "x": round(float(pts_rel[i, 0]), 2),
                "y": round(float(pts_rel[i, 1]), 2),
                "z": round(float(pts_rel[i, 2]), 2),
                "r": int(colors_rgb[::stride][i, 0] * 255),
                "g": int(colors_rgb[::stride][i, 1] * 255),
                "b": int(colors_rgb[::stride][i, 2] * 255),
            }
            for i in range(len(pts_rel))
        ]
    }
    json_file = OUT_DIR / "apple_points_sphere.json"
    with open(json_file, 'w') as f:
        json.dump(export_data, f)
    print(f"JSON data saved: {json_file}")

if __name__ == '__main__':
    main()
