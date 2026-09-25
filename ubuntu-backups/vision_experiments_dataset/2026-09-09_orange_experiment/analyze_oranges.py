#!/usr/bin/env python3
import json
from pathlib import Path
import cv2
import numpy as np
from ultralytics import YOLO

EXP_DIR = Path("/home/li/vision_experiments/2026-09-09_orange_experiment")
DATA_DIR = EXP_DIR / "data"

def sphere_fit_ransac(points, max_iters=250, inlier_thresh=3.0):
    if len(points) < 15:
        return None
    best_inliers = []
    best_sphere = None
    N = len(points)
    np.random.seed(42)
    for _ in range(max_iters):
        idx = np.random.choice(N, 4, replace=False)
        p = points[idx]
        A = 2 * (p[1:] - p[0])
        b = np.sum(p[1:]**2 - p[0]**2, axis=1)
        try:
            c = np.linalg.solve(A, b)
            r = np.linalg.norm(p[0] - c)
            if r < 15.0 or r > 65.0:  # mm
                continue
            dists = np.linalg.norm(points - c, axis=1)
            inliers = np.where(np.abs(dists - r) < inlier_thresh)[0]
            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_sphere = (c, r)
        except np.linalg.LinAlgError:
            continue
    if best_sphere is None or len(best_inliers) < 15:
        c = np.mean(points, axis=0)
        r = np.mean(np.linalg.norm(points - c, axis=1))
        return c, r, float(np.std(np.linalg.norm(points - c, axis=1))), len(points)
    c, r = best_sphere
    # Refine with least squares
    in_pts = points[best_inliers]
    A = 2 * (in_pts - np.mean(in_pts, axis=0))
    # Simple linear refinement
    dists = np.linalg.norm(in_pts - c, axis=1)
    rmse = float(np.sqrt(np.mean((dists - r)**2)))
    return c, r, rmse, len(best_inliers)

def main():
    color = cv2.imread(str(DATA_DIR / "orange_color.jpg"))
    depth = cv2.imread(str(DATA_DIR / "orange_depth.png"), cv2.IMREAD_UNCHANGED)
    with open(DATA_DIR / "camera_info.json") as f:
        ci = json.load(f)

    fx, fy = ci['k'][0], ci['k'][4]
    cx, cy = ci['k'][2], ci['k'][5]

    model = YOLO("/home/li/hand_eye_calibration/models/yolo11n-seg.pt")
    results = model(color, classes=[47, 49], conf=0.20, verbose=False)[0]

    detections = []
    annotated = color.copy()

    # Pre-define some colors for visualization
    palette = [
        (0, 165, 255),   # Orange
        (0, 200, 255),   # Yellow-orange
        (50, 205, 50),   # Lime
        (255, 144, 30),  # Blue-orange
        (0, 215, 255),   # Gold
        (180, 105, 255), # Pink
        (255, 191, 0),   # Deep sky
        (0, 128, 255),   # Pure orange
        (0, 0, 255),     # Red
    ]

    print("\n" + "="*80)
    print(f"{'ID':<4} {'Class':<8} {'Conf':<6} {'Radius(mm)':<12} {'RMSE(mm)':<10} {'Inliers':<10} {'Camera 3D Center [X, Y, Z] (mm)':<30}")
    print("="*80)

    for i, box in enumerate(results.boxes):
        cls_id = int(box.cls[0])
        cls_name = model.names[cls_id]
        conf = float(box.conf[0])
        x1, y1, x2, y2 = map(int, box.xyxy[0])

        mask = np.zeros(depth.shape[:2], dtype=bool)
        if results.masks is not None and len(results.masks.data) > i:
            m = results.masks.data[i].cpu().numpy()
            m = cv2.resize(m, (depth.shape[1], depth.shape[0])) > 0.5
            mask = m
        else:
            mask[y1:y2, x1:x2] = True

        ys, xs = np.nonzero(mask)
        if len(ys) == 0:
            continue
        d_vals = depth[ys, xs].astype(np.float64) * 0.001
        valid = (d_vals > 0.25) & (d_vals < 0.90)
        xs, ys, d_vals = xs[valid], ys[valid], d_vals[valid]

        if len(xs) < 15:
            continue

        X = (xs - cx) * d_vals / fx * 1000.0
        Y = (ys - cy) * d_vals / fy * 1000.0
        Z = d_vals * 1000.0
        pts = np.column_stack([X, Y, Z])

        fit_res = sphere_fit_ransac(pts)
        if fit_res is None:
            continue
        c, r, rmse, inlier_cnt = fit_res

        color_bgr = palette[i % len(palette)] if cls_name == "orange" else (0, 0, 255)
        # Overlay mask
        mask_overlay = annotated.copy()
        mask_overlay[mask] = color_bgr
        cv2.addWeighted(mask_overlay, 0.35, annotated, 0.65, 0, annotated)

        # Draw bbox & label
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color_bgr, 2)
        label_top = f"#{i} {cls_name} {conf:.2f}"
        label_bot = f"R={r:.1f}mm Z={c[2]:.0f}mm"

        cv2.putText(annotated, label_top, (x1, max(18, y1 - 18)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 3)
        cv2.putText(annotated, label_top, (x1, max(18, y1 - 18)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 1)

        cv2.putText(annotated, label_bot, (x1, max(32, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 3)
        cv2.putText(annotated, label_bot, (x1, max(32, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        # Draw 2D projection of 3D center
        u_c = int(c[0] * fx / c[2] + cx)
        v_c = int(c[1] * fy / c[2] + cy)
        cv2.drawMarker(annotated, (u_c, v_c), (0, 0, 255), cv2.MARKER_CROSS, 12, 2)

        c_str = f"[{c[0]:6.1f}, {c[1]:6.1f}, {c[2]:6.1f}]"
        print(f"#{i:<3} {cls_name:<8} {conf:<6.2f} {r:<12.1f} {rmse:<10.2f} {inlier_cnt:<10} {c_str:<30}")

        detections.append({
            "id": i,
            "class": cls_name,
            "conf": round(conf, 3),
            "radius_mm": round(float(r), 1),
            "rmse_mm": round(float(rmse), 2),
            "inliers": inlier_cnt,
            "camera_xyz_mm": [round(float(v), 1) for v in c],
            "bbox": [x1, y1, x2, y2]
        })

    out_img = EXP_DIR / "orange_detection_annotated.jpg"
    cv2.imwrite(str(out_img), annotated)
    print(f"\nAnnotated image saved: {out_img}")

    out_json = EXP_DIR / "orange_detections.json"
    with open(out_json, "w") as f:
        json.dump(detections, f, indent=2)
    print(f"Results JSON saved: {out_json}")

if __name__ == '__main__':
    main()
