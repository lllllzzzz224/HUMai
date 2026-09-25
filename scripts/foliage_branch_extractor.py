#!/usr/bin/env python3
"""
Foliage Branch Extractor (Scheme 2 + Scheme 3 Core Module)
---------------------------------------------------------
Extracts rigid branch collision cylinders under foliage occlusion:
1. Spatial ROI Truncation (bounds to approach corridor around fruit)
2. ExG Leaf Stripping (separates compliant soft leaves from rigid wood)
3. RANSAC Cylinder Fitting with Physical Radius Prior
4. MoveIt CollisionObject Generation with Safety Margin Inflation
"""

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class BranchCylinder:
    """Rigid branch cylinder model for MoveIt collision avoidance."""
    id: str
    center: List[float]       # [x, y, z] in base_link (midpoint of cylinder)
    direction: List[float]    # [dx, dy, dz] unit direction vector
    quaternion: List[float]   # [qx, qy, qz, qw] aligning [0, 0, 1] to direction
    radius_raw_m: float       # Fitted physical radius (meters)
    radius_inflated_m: float  # Radius with safety margin (meters)
    length_m: float           # Extent along axis (meters)
    inlier_count: int         # Supporting points
    confidence: float         # Quality score [0.0, 1.0]

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class FoliageBranchExtractor:
    """
    Decouples soft leaves from rigid branches and extracts 3D collision cylinders.
    """

    def __init__(
        self,
        roi_radius_m: float = 0.18,
        leaf_exg_thresh: float = 10.0,
        wood_exg_thresh: float = 0.0,
        fruit_exclude_radius_m: float = 0.045,
        min_branch_radius_m: float = 0.005,
        max_branch_radius_m: float = 0.025,
        margin_inflation_m: float = 0.010,
        min_inliers: int = 150,
    ):
        self.roi_radius_m = roi_radius_m
        self.leaf_exg_thresh = leaf_exg_thresh
        self.wood_exg_thresh = wood_exg_thresh
        self.fruit_exclude_radius_m = fruit_exclude_radius_m
        self.min_branch_radius_m = min_branch_radius_m
        self.max_branch_radius_m = max_branch_radius_m
        self.margin_inflation_m = margin_inflation_m
        self.min_inliers = min_inliers

    def extract(
        self,
        xyz_pts: np.ndarray,
        rgb_pts: np.ndarray,
        fruit_center: np.ndarray,
        max_cylinders: int = 2,
    ) -> Tuple[List[BranchCylinder], Dict[str, Any]]:
        """
        Main extraction pipeline.
        
        Args:
            xyz_pts: (N, 3) 3D coordinates in meters.
            rgb_pts: (N, 3) color values [0, 255].
            fruit_center: (3,) target fruit center [x, y, z].
            max_cylinders: Maximum number of rigid branch cylinders to return (Top-1 or Top-2).
            
        Returns:
            cylinders: List of BranchCylinder objects.
            stats: Diagnostic dictionary with point counts and timings.
        """
        assert xyz_pts.shape[0] == rgb_pts.shape[0], "Points and colors shape mismatch"
        total_pts = len(xyz_pts)
        if total_pts == 0:
            return [], {"total_points": 0, "error": "empty_cloud"}

        # Step 1: Spatial ROI Truncation around fruit
        dists_to_fruit = np.linalg.norm(xyz_pts - fruit_center, axis=1)
        roi_mask = dists_to_fruit <= self.roi_radius_m
        roi_pts = xyz_pts[roi_mask]
        roi_rgb = rgb_pts[roi_mask].astype(float)
        pts_in_roi = len(roi_pts)

        if pts_in_roi < self.min_inliers:
            return [], {
                "total_points": total_pts,
                "roi_points": pts_in_roi,
                "status": "too_few_roi_points",
            }

        # Step 2: Leaf Stripping via Excess Green (ExG)
        # ExG = 2 * Green - Red - Blue
        exg = 2.0 * roi_rgb[:, 1] - roi_rgb[:, 0] - roi_rgb[:, 2]
        leaves_mask = exg > self.leaf_exg_thresh
        wood_mask = exg <= self.wood_exg_thresh

        # Exclude points too close to fruit center (avoid fitting fruit skin as branch)
        non_fruit_mask = dists_to_fruit[roi_mask] > self.fruit_exclude_radius_m
        branch_candidates = roi_pts[wood_mask & non_fruit_mask]

        stats = {
            "total_points": total_pts,
            "roi_points": pts_in_roi,
            "leaf_points_stripped": int(np.sum(leaves_mask)),
            "wood_candidate_points": len(branch_candidates),
            "cylinders_found": 0,
        }

        if len(branch_candidates) < self.min_inliers:
            stats["status"] = "no_significant_branch_found"
            return [], stats

        # Step 3: Cylinder Fitting
        cylinders: List[BranchCylinder] = []
        remaining_pts = np.copy(branch_candidates)

        for cyl_idx in range(max_cylinders):
            if len(remaining_pts) < self.min_inliers:
                break

            cyl, inlier_indices = self._fit_single_cylinder(
                remaining_pts, cyl_id=f"branch_cylinder_{cyl_idx}"
            )
            if cyl is None:
                break

            cylinders.append(cyl)
            # Remove inliers for next iteration
            mask_keep = np.ones(len(remaining_pts), dtype=bool)
            mask_keep[inlier_indices] = False
            remaining_pts = remaining_pts[mask_keep]

        stats["cylinders_found"] = len(cylinders)
        stats["status"] = "success"
        return cylinders, stats

    def _fit_single_cylinder(
        self, pts: np.ndarray, cyl_id: str
    ) -> Tuple[Optional[BranchCylinder], np.ndarray]:
        """
        Fit a single cylinder using PCA axis estimation + perpendicular distance evaluation.
        """
        if len(pts) < self.min_inliers:
            return None, np.array([])

        centroid = np.mean(pts, axis=0)
        centered = pts - centroid

        # PCA for principal direction
        cov = np.cov(centered, rowvar=False)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        idx = np.argsort(eigenvalues)[::-1]
        eigenvalues = eigenvalues[idx]
        eigenvectors = eigenvectors[:, idx]

        direction = eigenvectors[:, 0]
        direction /= np.linalg.norm(direction)

        # Linearity check: lambda_1 should dominate lambda_2
        linearity = eigenvalues[0] / (eigenvalues[1] + 1e-6)
        if linearity < 2.0:
            # Not elongated enough to be a branch
            return None, np.array([])

        # Project points onto axis
        proj_len = np.dot(centered, direction)
        perp_vecs = centered - np.outer(proj_len, direction)
        perp_dists = np.linalg.norm(perp_vecs, axis=1)

        # Estimate radius as median of perpendicular distances
        radius_est = float(np.median(perp_dists))

        # Clamp / filter by physical branch radius prior
        if radius_est < self.min_branch_radius_m or radius_est > self.max_branch_radius_m:
            return None, np.array([])

        # Inlier evaluation (within 8mm of cylinder surface)
        dist_to_surface = np.abs(perp_dists - radius_est)
        inlier_mask = dist_to_surface <= 0.008
        inlier_count = int(np.sum(inlier_mask))

        if inlier_count < self.min_inliers:
            return None, np.array([])

        # Calculate bounding extent along axis based on inliers
        inlier_proj = proj_len[inlier_mask]
        t_min = float(np.percentile(inlier_proj, 1.0))
        t_max = float(np.percentile(inlier_proj, 99.0))
        length = float(max(0.05, t_max - t_min))

        # Midpoint of cylinder
        t_mid = (t_min + t_max) / 2.0
        center_mid = centroid + t_mid * direction

        # Quaternion: align ROS cylinder convention [0, 0, 1] with branch direction
        z_axis = np.array([0.0, 0.0, 1.0])
        cross_prod = np.cross(z_axis, direction)
        cross_norm = np.linalg.norm(cross_prod)
        dot_prod = float(np.dot(z_axis, direction))

        if cross_norm > 1e-6:
            rot_axis = cross_prod / cross_norm
            rot_angle = math.acos(np.clip(dot_prod, -1.0, 1.0))
            rot = Rotation.from_rotvec(rot_axis * rot_angle)
        elif dot_prod > 0:
            rot = Rotation.identity()
        else:
            rot = Rotation.from_rotvec(np.array([1.0, 0.0, 0.0]) * math.pi)

        quat = [float(q) for q in rot.as_quat()]  # [qx, qy, qz, qw]

        confidence = float(min(1.0, (inlier_count / len(pts)) * (linearity / 5.0)))

        cylinder = BranchCylinder(
            id=cyl_id,
            center=[float(c) for c in center_mid],
            direction=[float(d) for d in direction],
            quaternion=quat,
            radius_raw_m=radius_est,
            radius_inflated_m=radius_est + self.margin_inflation_m,
            length_m=length,
            inlier_count=inlier_count,
            confidence=confidence,
        )

        inlier_indices = np.where(inlier_mask)[0]
        return cylinder, inlier_indices


def to_moveit_msg(cylinder: BranchCylinder, frame_id: str = "base_link") -> Dict[str, Any]:
    """
    Converts a BranchCylinder to a moveit_msgs/CollisionObject dictionary format.
    """
    return {
        "header": {"frame_id": frame_id},
        "id": cylinder.id,
        "operation": 0,  # ADD
        "primitives": [
            {
                "type": 3,  # CYLINDER
                "dimensions": [cylinder.length_m, cylinder.radius_inflated_m],
            }
        ],
        "primitive_poses": [
            {
                "position": {
                    "x": cylinder.center[0],
                    "y": cylinder.center[1],
                    "z": cylinder.center[2],
                },
                "orientation": {
                    "x": cylinder.quaternion[0],
                    "y": cylinder.quaternion[1],
                    "z": cylinder.quaternion[2],
                    "w": cylinder.quaternion[3],
                },
            }
        ],
    }


def main():
    parser = argparse.ArgumentParser(description="Extract rigid branch cylinders under foliage.")
    parser.add_argument("--npz", type=str, required=True, help="Path to observed_points_base.npz")
    parser.add_argument(
        "--fruit-center",
        type=float,
        nargs=3,
        default=[0.3534, -0.2419, -0.0681],
        help="Target fruit center (x, y, z) in meters",
    )
    parser.add_argument("--output", type=str, default="branch_cylinders.json", help="Output JSON path")
    args = parser.parse_args()

    npz_path = Path(args.npz)
    if not npz_path.exists():
        print(f"Error: {npz_path} does not exist", file=sys.stderr)
        sys.exit(1)

    data = np.load(npz_path)
    xyz_pts = data["xyz_m"]
    rgb_pts = data["rgb"]
    fruit_center = np.array(args.fruit_center)

    extractor = FoliageBranchExtractor()
    cylinders, stats = extractor.extract(xyz_pts, rgb_pts, fruit_center)

    print("=== Foliage Branch Extractor Results ===")
    print(f"Total points: {stats['total_points']}")
    print(f"Points in 18cm ROI: {stats['roi_points']}")
    print(f"Leaf points stripped: {stats['leaf_points_stripped']}")
    print(f"Wood/Branch candidate points: {stats['wood_candidate_points']}")
    print(f"Cylinders found: {stats['cylinders_found']}")

    output_data = {
        "fruit_center": list(fruit_center),
        "stats": stats,
        "cylinders": [c.to_dict() for c in cylinders],
        "moveit_objects": [to_moveit_msg(c) for c in cylinders],
    }

    for i, c in enumerate(cylinders):
        print(f"\n[Cylinder #{i+1}] {c.id}:")
        print(f"  Center: [{c.center[0]:.4f}, {c.center[1]:.4f}, {c.center[2]:.4f}] m")
        print(f"  Direction: [{c.direction[0]:.4f}, {c.direction[1]:.4f}, {c.direction[2]:.4f}]")
        print(f"  Radius (raw/inflated): {c.radius_raw_m*1000:.1f} mm / {c.radius_inflated_m*1000:.1f} mm")
        print(f"  Length: {c.length_m*1000:.1f} mm")
        print(f"  Inliers: {c.inlier_count}, Confidence: {c.confidence:.2f}")

    out_path = Path(args.output)
    out_path.write_text(json.dumps(output_data, indent=2))
    print(f"\nSaved cylinder models to: {out_path}")


if __name__ == "__main__":
    main()
