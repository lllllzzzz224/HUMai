#!/usr/bin/env python3
"""Detect and localize apples in the branch occlusion scene."""

from __future__ import annotations

import json
from pathlib import Path
import cv2
import numpy as np
from scipy.optimize import least_squares
from ultralytics import YOLO

DATA_DIR = Path("/home/li/vision_experiments/2026-09-09_branch_occlusion/data")
MODEL_PATH = Path("/home/li/hand_eye_calibration/models/yolo11n-seg.pt")
APPLE_CLASS_ID = 47

def estimate_method_a(mask, xyxy, depth, fx, fy, cx, cy):
    ys, xs = np.nonzero(mask)
    if xs.size < 50:
        return None
    x0, y0, x1, y1 = xyxy
    width_px = max(float(x1 - x0), 1.0)
    height_px = max(float(y1 - y0), 1.0)
    u = float(np.median(xs))
    v = float(np.median(ys))

    h_img, w_img = depth.shape
    y_grid, x_grid = np.indices((h_img, w_img))
    central = (
        mask
        & (x_grid >= u - 0.22 * width_px)
        & (x_grid <= u + 0.22 * width_px)
        & (y_grid >= v - 0.22 * height_px)
        & (y_grid <= v + 0.22 * height_px)
    )
    values = depth[central]
    values = values[(values > 100) & (values < 3000)]
    if values.size < 20:
        return None
    z_surface = float(np.median(values)) * 0.001
    ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=np.float64)
    ray_unit = ray / np.linalg.norm(ray)
    surface = ray * z_surface
    radius_x = 0.5 * width_px * z_surface / fx
    radius_y = 0.5 * height_px * z_surface / fy
    radius_m = float(np.median([radius_x, radius_y]))
    center = surface + ray_unit * radius_m
    return {
        "surface": surface.tolist(),
        "center": center.tolist(),
        "radius_mm": radius_m * 1000.0,
    }

def fit_sphere_algebraic(pts):
    M = np.column_stack([pts, np.ones(pts.shape[0])])
    f = np.sum(pts**2, axis=1)
    try:
        sol, residuals, rank, s = np.linalg.lstsq(M, f, rcond=1e-4)
        A, B, C, D = sol
        x0 = A / 2.0
        y0 = B / 2.0
        z0 = C / 2.0
        r_sq = x0**2 + y0**2 + z0**2 + D
        if r_sq <= 0:
            return None
        return np.array([x0, y0, z0]), np.sqrt(r_sq)
    except Exception:
        return None

def estimate_method_b(mask, depth, fx, fy, cx, cy):
    kernel = np.ones((3, 3), np.uint8)
    eroded = cv2.erode(mask.astype(np.uint8), kernel, iterations=1).astype(bool)
    ys, xs = np.nonzero(eroded)
    if xs.size < 40:
        ys, xs = np.nonzero(mask)
        if xs.size < 40:
            return None

    d_vals = depth[ys, xs].astype(np.float64) * 0.001
    valid = (d_vals > 0.1) & (d_vals < 2.0)
    xs, ys, d_vals = xs[valid], ys[valid], d_vals[valid]
    if xs.size < 40:
        return None

    med_z = np.median(d_vals)
    cluster = np.abs(d_vals - med_z) < 0.050
    xs, ys, d_vals = xs[cluster], ys[cluster], d_vals[cluster]
    if xs.size < 30:
        return None

    X = (xs - cx) * d_vals / fx
    Y = (ys - cy) * d_vals / fy
    Z = d_vals
    pts = np.column_stack([X, Y, Z])

    rng = np.random.default_rng(42)
    best_inliers, best_c, best_r = None, None, None
    n_pts = pts.shape[0]

    for _ in range(250):
        sample_indices = rng.choice(n_pts, size=4, replace=False)
        fit = fit_sphere_algebraic(pts[sample_indices])
        if fit is None:
            continue
        c, r = fit
        if not (0.020 <= r <= 0.060):
            continue
        if c[2] <= med_z - 0.010 or c[2] >= med_z + r + 0.025:
            continue
        resids = np.abs(np.linalg.norm(pts - c, axis=1) - r)
        inliers = resids < 0.005
        if best_inliers is None or np.sum(inliers) > np.sum(best_inliers):
            best_inliers = inliers
            best_c = c
            best_r = r

    if best_inliers is None or np.sum(best_inliers) < 25:
        return None

    inlier_pts = pts[best_inliers]
    def sphere_residuals(params, p):
        xc, yc, zc, rad = params
        d = np.sqrt((p[:, 0] - xc)**2 + (p[:, 1] - yc)**2 + (p[:, 2] - zc)**2)
        return d - rad

    p0 = [best_c[0], best_c[1], best_c[2], best_r]
    bounds = (
        [-np.inf, -np.inf, med_z - 0.005, 0.020],
        [ np.inf,  np.inf, med_z + 0.08, 0.060]
    )
    res = least_squares(sphere_residuals, p0, bounds=bounds, args=(inlier_pts,), loss='soft_l1', f_scale=0.002)
    refined_c = res.x[:3]
    refined_r = res.x[3]
    dists = np.linalg.norm(inlier_pts - refined_c, axis=1)
    rmse_mm = float(np.sqrt(np.mean((dists - refined_r)**2)) * 1000.0)

    return {
        "center": refined_c.tolist(),
        "radius_mm": refined_r * 1000.0,
        "rmse_mm": rmse_mm,
        "inliers": int(np.sum(best_inliers)),
    }

def main():
    with open(DATA_DIR / "camera_info.json") as f:
        cam_info = json.load(f)
    fx = cam_info["k"][0]
    fy = cam_info["k"][4]
    cx = cam_info["k"][2]
    cy = cam_info["k"][5]

    color = cv2.imread(str(DATA_DIR / "branch_color.jpg"))
    depth = cv2.imread(str(DATA_DIR / "branch_depth.png"), cv2.IMREAD_UNCHANGED)
    model = YOLO(str(MODEL_PATH))

    # Tiled inference to catch distant/small fruits with branches
    height, width = color.shape[:2]
    tile_width = 480
    starts = [0, 80, 160]
    
    raw_candidates = []
    for x0 in starts:
        tile = color[:, x0:x0+tile_width]
        res = model(tile, conf=0.10, verbose=False)[0]
        if res.boxes is None or res.masks is None:
            continue
        for b_idx in range(len(res.boxes)):
            if int(res.boxes.cls[b_idx]) != APPLE_CLASS_ID:
                continue
            conf = float(res.boxes.conf[b_idx])
            loc_xyxy = res.boxes.xyxy[b_idx].cpu().numpy()
            glob_xyxy = loc_xyxy.copy()
            glob_xyxy[0] += x0
            glob_xyxy[2] += x0

            poly = res.masks.xy[b_idx]
            tile_mask = np.zeros(tile.shape[:2], dtype=np.uint8)
            if len(poly) >= 3:
                cv2.fillPoly(tile_mask, [np.int32(poly)], 1)
            glob_mask = np.zeros(depth.shape[:2], dtype=bool)
            glob_mask[:, x0:x0+tile_width] = tile_mask.astype(bool)

            raw_candidates.append({
                "conf": conf,
                "xyxy": glob_xyxy,
                "mask": glob_mask,
                "x0": x0
            })

    # NMS / deduplicate candidates based on 2D IoU
    def iou(b1, b2):
        x0 = max(b1[0], b2[0])
        y0 = max(b1[1], b2[1])
        x1 = min(b1[2], b2[2])
        y1 = min(b1[3], b2[3])
        inter = max(0, x1 - x0) * max(0, y1 - y0)
        area1 = (b1[2] - b1[0]) * (b1[3] - b1[1])
        area2 = (b2[2] - b2[0]) * (b2[3] - b2[1])
        return inter / max(1e-5, area1 + area2 - inter)

    deduped = []
    for cand in sorted(raw_candidates, key=lambda c: c["conf"], reverse=True):
        if not any(iou(cand["xyxy"], d["xyxy"]) > 0.4 for d in deduped):
            deduped.append(cand)

    print(f"Total distinct apples detected amidst branches: {len(deduped)}")
    annotated = color.copy()

    for idx, target in enumerate(deduped):
        conf = target["conf"]
        xyxy = target["xyxy"]
        mask = target["mask"]
        x0, y0, x1, y1 = [int(v) for v in xyxy]

        res_a = estimate_method_a(mask, xyxy, depth, fx, fy, cx, cy)
        res_b = estimate_method_b(mask, depth, fx, fy, cx, cy)

        # Draw on image
        cv2.rectangle(annotated, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2.putText(annotated, f"Apple #{idx} conf:{conf:.2f}", (x0, max(y0 - 6, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        print(f"\n--- Apple #{idx} (conf={conf:.3f}, bbox=[{x0}, {y0}, {x1}, {y1}]) ---")
        if res_a:
            ca = res_a["center"]
            ra = res_a["radius_mm"]
            print(f"  Method A (Heuristic): Center=[{ca[0]*1000:.1f}, {ca[1]*1000:.1f}, {ca[2]*1000:.1f}] mm, R={ra:.1f} mm")
            cv2.putText(annotated, f"A: R={ra:.0f} Z={ca[2]*1000:.0f}", (x0, y1 + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 165, 255), 1)
        else:
            print("  Method A: FAILED")

        if res_b:
            cb = res_b["center"]
            rb = res_b["radius_mm"]
            rmse = res_b["rmse_mm"]
            print(f"  Method B (SphereFit): Center=[{cb[0]*1000:.1f}, {cb[1]*1000:.1f}, {cb[2]*1000:.1f}] mm, R={rb:.1f} mm, RMSE={rmse:.2f} mm")
            cv2.putText(annotated, f"B: R={rb:.0f} Z={cb[2]*1000:.0f} (RMSE={rmse:.1f})", (x0, y1 + 32), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 50, 50), 1)
        else:
            print("  Method B: FAILED")

    out_annotated = DATA_DIR / "branch_annotated.jpg"
    cv2.imwrite(str(out_annotated), annotated)
    print(f"\nAnnotated image saved: {out_annotated}")

if __name__ == "__main__":
    main()
