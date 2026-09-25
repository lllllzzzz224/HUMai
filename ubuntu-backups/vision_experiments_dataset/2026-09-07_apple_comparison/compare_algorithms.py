#!/usr/bin/env python3
"""Compare Method A (Heuristic Ray Offset) vs Method B (RANSAC Sphere Fit) with Spatial Tracking."""

from __future__ import annotations

import json
from pathlib import Path
import time
import cv2
import numpy as np
from scipy.optimize import least_squares
from ultralytics import YOLO

EXP_DIR = Path("/home/li/vision_experiments/2026-09-07_apple_comparison")
DATA_DIR = EXP_DIR / "data"
MODEL_PATH = Path("/home/li/hand_eye_calibration/models/yolo11n-seg.pt")
APPLE_CLASS_ID = 47

# ----------------- Method A: Heuristic (Existing Localizer v2) -----------------
def estimate_method_a(mask: np.ndarray, xyxy: np.ndarray, depth: np.ndarray, fx: float, fy: float, cx: float, cy: float):
    ys, xs = np.nonzero(mask)
    if xs.size < 50:
        return None, "mask_too_small"

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
    if values.size < 30:
        return None, "insufficient_depth"

    z_surface = float(np.median(values)) * 0.001
    ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=np.float64)
    ray_unit = ray / np.linalg.norm(ray)
    surface = ray * z_surface

    radius_x = 0.5 * width_px * z_surface / fx
    radius_y = 0.5 * height_px * z_surface / fy
    radius_m = float(np.median([radius_x, radius_y]))

    if not 0.020 <= radius_m <= 0.080:
        return None, f"implausible_radius_{radius_m*1000:.1f}mm"

    center = surface + ray_unit * radius_m
    return {
        "method": "A_Heuristic",
        "surface": surface.tolist(),
        "center": center.tolist(),
        "radius_mm": radius_m * 1000.0,
        "valid_points": int(values.size),
        "rmse_mm": None,
        "inlier_ratio": None,
    }, "ok"


# ----------------- Method B: RANSAC Sphere Fit with Refinement -----------------
def fit_sphere_algebraic(pts: np.ndarray):
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

def estimate_method_b(mask: np.ndarray, depth: np.ndarray, fx: float, fy: float, cx: float, cy: float,
                       r_min_m=0.025, r_max_m=0.055, inlier_thresh_m=0.004, max_iters=250):
    kernel = np.ones((3, 3), np.uint8)
    eroded_mask = cv2.erode(mask.astype(np.uint8), kernel, iterations=1).astype(bool)

    ys, xs = np.nonzero(eroded_mask)
    if xs.size < 50:
        ys, xs = np.nonzero(mask)
        if xs.size < 50:
            return None, "mask_too_small"

    d_vals = depth[ys, xs].astype(np.float64) * 0.001
    valid = (d_vals > 0.1) & (d_vals < 2.0)
    xs, ys, d_vals = xs[valid], ys[valid], d_vals[valid]
    if xs.size < 50:
        return None, "insufficient_depth"

    med_z = np.median(d_vals)
    cluster = np.abs(d_vals - med_z) < 0.050
    xs, ys, d_vals = xs[cluster], ys[cluster], d_vals[cluster]
    if xs.size < 40:
        return None, "insufficient_depth_cluster"

    X = (xs - cx) * d_vals / fx
    Y = (ys - cy) * d_vals / fy
    Z = d_vals
    pts = np.column_stack([X, Y, Z])

    rng = np.random.default_rng(42)
    best_inliers = None
    best_center = None
    best_radius = None
    n_pts = pts.shape[0]

    for _ in range(max_iters):
        sample_indices = rng.choice(n_pts, size=4, replace=False)
        sample = pts[sample_indices]
        fit = fit_sphere_algebraic(sample)
        if fit is None:
            continue
        c, r = fit
        if not (r_min_m <= r <= r_max_m):
            continue
        if c[2] <= med_z - 0.010 or c[2] >= med_z + r + 0.020:
            continue

        dists = np.linalg.norm(pts - c, axis=1)
        resids = np.abs(dists - r)
        inliers = resids < inlier_thresh_m
        inlier_count = np.sum(inliers)

        if best_inliers is None or inlier_count > np.sum(best_inliers):
            best_inliers = inliers
            best_center = c
            best_radius = r

    if best_inliers is None or np.sum(best_inliers) < 30:
        return None, "ransac_fit_failed"

    inlier_pts = pts[best_inliers]

    def sphere_residuals(params, p):
        xc, yc, zc, rad = params
        d = np.sqrt((p[:, 0] - xc)**2 + (p[:, 1] - yc)**2 + (p[:, 2] - zc)**2)
        return d - rad

    p0 = [best_center[0], best_center[1], best_center[2], best_radius]
    bounds = (
        [-np.inf, -np.inf, med_z - 0.005, r_min_m],
        [ np.inf,  np.inf, med_z + r_max_m + 0.02, r_max_m]
    )
    res = least_squares(sphere_residuals, p0, bounds=bounds, args=(inlier_pts,), loss='soft_l1', f_scale=0.002)
    refined_c = res.x[:3]
    refined_r = res.x[3]

    final_dists = np.linalg.norm(inlier_pts - refined_c, axis=1)
    rmse_mm = float(np.sqrt(np.mean((final_dists - refined_r)**2)) * 1000.0)
    inlier_ratio = float(np.sum(best_inliers) / n_pts)

    closest_idx = np.argmin(np.linalg.norm(inlier_pts - refined_c, axis=1))
    surface_pt = inlier_pts[closest_idx]

    return {
        "method": "B_RANSAC_Sphere",
        "surface": surface_pt.tolist(),
        "center": refined_c.tolist(),
        "radius_mm": refined_r * 1000.0,
        "valid_points": int(n_pts),
        "inliers_count": int(np.sum(best_inliers)),
        "inlier_ratio": inlier_ratio,
        "rmse_mm": rmse_mm,
    }, "ok"


def main():
    with open(DATA_DIR / "camera_info.json") as f:
        cam_info = json.load(f)
    fx = cam_info["k"][0]
    fy = cam_info["k"][4]
    cx = cam_info["k"][2]
    cy = cam_info["k"][5]

    model = YOLO(str(MODEL_PATH))

    color_files = sorted(DATA_DIR.glob("frame_*_color.jpg"))
    print(f"Loaded {len(color_files)} frames for comparison.\n")

    # Spatial cluster targets: Left apple (~u=270, v=175), Right apple (~u=540, v=140)
    TARGET_CLUSTERS = {
        "Apple_Left": np.array([271.0, 175.0]),
        "Apple_Right": np.array([542.0, 139.0])
    }

    stats = {
        target_name: {
            "detections": 0,
            "conf_list": [],
            "method_a_centers": [],
            "method_a_radii": [],
            "method_b_centers": [],
            "method_b_radii": [],
            "method_b_rmse": [],
            "method_b_inliers": [],
            "time_a_ms": [],
            "time_b_ms": []
        }
        for target_name in TARGET_CLUSTERS
    }

    results_all = []
    annotated_sample = None

    for frame_idx, c_path in enumerate(color_files):
        d_path = c_path.parent / (c_path.stem.replace("color", "depth") + ".png")
        color = cv2.imread(str(c_path))
        depth = cv2.imread(str(d_path), cv2.IMREAD_UNCHANGED)

        t0 = time.perf_counter()
        # Use lower conf threshold (0.15) to match default MIN_CONFIDENCE in apple_center_localizer_v2.py
        yolo_res = model(color, conf=0.15, verbose=False)[0]
        t_yolo = (time.perf_counter() - t0) * 1000.0

        boxes = yolo_res.boxes
        masks = yolo_res.masks

        frame_record = {"frame_index": frame_idx, "yolo_time_ms": t_yolo, "apples": {}}

        if frame_idx == 0:
            annotated_sample = color.copy()

        if boxes is not None and masks is not None:
            for b_i in range(len(boxes)):
                cls_id = int(boxes.cls[b_i])
                if cls_id != APPLE_CLASS_ID:
                    continue

                conf = float(boxes.conf[b_i])
                xyxy = boxes.xyxy[b_i].cpu().numpy()
                u_c = 0.5 * (xyxy[0] + xyxy[2])
                v_c = 0.5 * (xyxy[1] + xyxy[3])
                center_2d = np.array([u_c, v_c])

                # Match to spatial cluster
                matched_target = min(TARGET_CLUSTERS.keys(), key=lambda name: np.linalg.norm(center_2d - TARGET_CLUSTERS[name]))
                if np.linalg.norm(center_2d - TARGET_CLUSTERS[matched_target]) > 100.0:
                    continue  # spurious detection outside known locations

                mask_poly = masks.xy[b_i]
                mask = np.zeros(depth.shape, dtype=np.uint8)
                if len(mask_poly) >= 3:
                    cv2.fillPoly(mask, [np.int32(mask_poly)], 1)
                mask = mask.astype(bool)

                t_a0 = time.perf_counter()
                res_a, status_a = estimate_method_a(mask, xyxy, depth, fx, fy, cx, cy)
                t_a = (time.perf_counter() - t_a0) * 1000.0

                t_b0 = time.perf_counter()
                res_b, status_b = estimate_method_b(mask, depth, fx, fy, cx, cy)
                t_b = (time.perf_counter() - t_b0) * 1000.0

                stats[matched_target]["detections"] += 1
                stats[matched_target]["conf_list"].append(conf)
                stats[matched_target]["time_a_ms"].append(t_a)
                stats[matched_target]["time_b_ms"].append(t_b)

                if res_a is not None:
                    stats[matched_target]["method_a_centers"].append(res_a["center"])
                    stats[matched_target]["method_a_radii"].append(res_a["radius_mm"])

                if res_b is not None:
                    stats[matched_target]["method_b_centers"].append(res_b["center"])
                    stats[matched_target]["method_b_radii"].append(res_b["radius_mm"])
                    stats[matched_target]["method_b_rmse"].append(res_b["rmse_mm"])
                    stats[matched_target]["method_b_inliers"].append(res_b["inliers_count"])

                frame_record["apples"][matched_target] = {
                    "conf": conf,
                    "bbox": [round(float(v), 1) for v in xyxy],
                    "method_a": res_a,
                    "method_b": res_b,
                }

                # Annotate frame 0
                if frame_idx == 0 and annotated_sample is not None:
                    x0, y0, x1, y1 = [int(v) for v in xyxy]
                    cv2.rectangle(annotated_sample, (x0, y0), (x1, y1), (0, 255, 0), 2)
                    label = f"{matched_target} conf:{conf:.2f}"
                    cv2.putText(annotated_sample, label, (x0, max(y0 - 6, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    if res_a and res_b:
                        info_a = f"A: R={res_a['radius_mm']:.1f}mm Z={res_a['center'][2]*1000:.1f}mm"
                        info_b = f"B: R={res_b['radius_mm']:.1f}mm Z={res_b['center'][2]*1000:.1f}mm (RMSE={res_b['rmse_mm']:.2f})"
                        cv2.putText(annotated_sample, info_a, (x0, y1 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 165, 255), 1)
                        cv2.putText(annotated_sample, info_b, (x0, y1 + 34), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 50, 50), 1)

        results_all.append(frame_record)

    if annotated_sample is not None:
        vis_path = EXP_DIR / "comparison_annotated_frame0.jpg"
        cv2.imwrite(str(vis_path), annotated_sample)
        print(f"Visualized frame saved: {vis_path}")

    out_file = EXP_DIR / "comparison_tracked_results.json"
    with open(out_file, "w") as f:
        json.dump(results_all, f, indent=2)

    total_frames = len(color_files)
    print("=" * 90)
    print(f"{'Target':<14} | {'Recall':<8} | {'AvgConf':<8} | {'Method':<20} | {'Center [X, Y, Z] (mm)':<26} | {'Radius (mm)'}")
    print("=" * 90)

    for target_name, data in stats.items():
        det_cnt = data["detections"]
        recall = det_cnt / total_frames
        avg_conf = np.mean(data["conf_list"]) if data["conf_list"] else 0.0

        ca = np.array(data["method_a_centers"]) * 1000.0 if data["method_a_centers"] else None
        cb = np.array(data["method_b_centers"]) * 1000.0 if data["method_b_centers"] else None
        ra = np.array(data["method_a_radii"]) if data["method_a_radii"] else None
        rb = np.array(data["method_b_radii"]) if data["method_b_radii"] else None

        print(f"\nTarget: {target_name}")
        print(f"  Detection Recall: {det_cnt}/{total_frames} ({recall:.1%}), Avg Confidence: {avg_conf:.3f}")
        
        if ca is not None and len(ca) > 0:
            mean_ca = np.mean(ca, axis=0)
            std_ca = np.std(ca, axis=0)
            mean_ra = np.mean(ra)
            std_ra = np.std(ra)
            avg_ta = np.mean(data["time_a_ms"])
            print(f"  Method A (Heuristic Ray):")
            print(f"    Center: [{mean_ca[0]:.1f}, {mean_ca[1]:.1f}, {mean_ca[2]:.1f}] mm (Std: [{std_ca[0]:.2f}, {std_ca[1]:.2f}, {std_ca[2]:.2f}] mm)")
            print(f"    Radius: {mean_ra:.1f} mm (Std: {std_ra:.2f} mm)")
            print(f"    Latency: {avg_ta:.2f} ms")

        if cb is not None and len(cb) > 0:
            mean_cb = np.mean(cb, axis=0)
            std_cb = np.std(cb, axis=0)
            mean_rb = np.mean(rb)
            std_rb = np.std(rb)
            mean_rmse = np.mean(data["method_b_rmse"])
            mean_inliers = np.mean(data["method_b_inliers"])
            avg_tb = np.mean(data["time_b_ms"])
            print(f"  Method B (RANSAC Sphere Fit):")
            print(f"    Center: [{mean_cb[0]:.1f}, {mean_cb[1]:.1f}, {mean_cb[2]:.1f}] mm (Std: [{std_cb[0]:.2f}, {std_cb[1]:.2f}, {std_cb[2]:.2f}] mm)")
            print(f"    Radius: {mean_rb:.1f} mm (Std: {std_rb:.2f} mm), Avg Inliers: {mean_inliers:.0f}, Avg RMSE: {mean_rmse:.2f} mm")
            print(f"    Latency: {avg_tb:.2f} ms")

        if ca is not None and cb is not None and len(ca) > 0 and len(cb) > 0:
            diff = mean_cb - mean_ca
            print(f"  >>> Systematic Bias (B - A): ΔX = {diff[0]:+.2f} mm, ΔY = {diff[1]:+.2f} mm, ΔZ = {diff[2]:+.2f} mm | Total 3D Distance = {np.linalg.norm(diff):.2f} mm")

    print("\n" + "=" * 90)

if __name__ == "__main__":
    main()
