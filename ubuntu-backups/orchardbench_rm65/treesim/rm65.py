"""Stage 3.1 fixed-base RM65 loading and static scene diagnostics.

This module deliberately contains no controller, camera, IK, picker, or RL
integration.  The robot model is the generated Stage 3A plain URDF; its source
of truth remains ``tools/generate_rm65_newton_urdf.py``.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import newton
import numpy as np
import warp as wp


REPO_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = REPO_ROOT / "assets" / "robots" / "rm65" / "rm65_with_gripper.urdf"

# Stage 3.1B inspection mount.  This is intentionally a scene-loading pose, not
# a reachability calibration: the base sits on z=0, 1.8 m from the tree origin,
# and yaw=pi faces the arm's +X direction toward the tree.  It is centralized
# here so later stages can replace it without scattering frame constants.
FIXED_MOUNT_XYZ = (1.8, 0.0, 0.0)
FIXED_MOUNT_RPY = (0.0, 0.0, math.pi)
FIXED_MOUNT_SOURCE = "Stage 3.1B static inspection pose; not a harvesting calibration"

ARM_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
GRIPPER_MOVABLE_JOINT_NAMES = ("gripper_left_joint", "gripper_right_joint")
GRIPPER_FIXED_JOINT_NAMES = (
    "gripper_mount_fixed",
    "gripper_palm_fixed",
    "gripper_tcp_fixed",
)
REQUIRED_LINK_NAMES = (
    "base_link",
    "Link1",
    "Link2",
    "Link3",
    "Link4",
    "Link5",
    "Link6",
    "gripper_mount",
    "gripper_palm",
    "gripper_left_finger",
    "gripper_right_finger",
    "gripper_tcp",
)


@dataclass(frozen=True)
class JointSpec:
    name: str
    kind: str
    parent: str
    child: str
    lower: float | None
    upper: float | None
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]


def _vector(element: ET.Element | None, attribute: str) -> tuple[float, float, float]:
    text = "0 0 0" if element is None else element.get(attribute, "0 0 0")
    values = tuple(float(value) for value in text.split())
    if len(values) != 3:
        raise ValueError(f"expected three values in {attribute}, got {text!r}")
    return values


def _short(label: object) -> str:
    return str(label).rsplit("/", 1)[-1]


def _unique_named_index(labels: list[object], name: str, start: int = 0) -> int:
    matches = [index for index in range(start, len(labels)) if _short(labels[index]) == name]
    if len(matches) != 1:
        raise ValueError(f"expected one Newton object named {name!r}, found {len(matches)}")
    return matches[0]


def inspect_urdf(path: Path = URDF_PATH) -> dict:
    """Validate the generated URDF and return its name-based structure."""
    root = ET.parse(path).getroot()
    links = [element.get("name", "") for element in root.findall("link")]
    if len(links) != len(set(links)):
        raise ValueError("RM65 URDF contains duplicate link names")
    missing_links = sorted(set(REQUIRED_LINK_NAMES) - set(links))
    if missing_links:
        raise ValueError(f"RM65 URDF is missing links: {missing_links}")
    if links.count("gripper_tcp") != 1:
        raise ValueError("gripper_tcp must occur exactly once in the RM65 URDF")

    specs: dict[str, JointSpec] = {}
    for joint in root.findall("joint"):
        name = joint.get("name", "")
        if name in specs:
            raise ValueError(f"duplicate RM65 joint name: {name}")
        limit = joint.find("limit")
        origin = joint.find("origin")
        specs[name] = JointSpec(
            name=name,
            kind=joint.get("type", ""),
            parent=joint.find("parent").get("link", ""),
            child=joint.find("child").get("link", ""),
            lower=(float(limit.get("lower")) if limit is not None and limit.get("lower") is not None else None),
            upper=(float(limit.get("upper")) if limit is not None and limit.get("upper") is not None else None),
            xyz=_vector(origin, "xyz"),
            rpy=_vector(origin, "rpy"),
        )

    expected_joints = set(ARM_JOINT_NAMES + GRIPPER_MOVABLE_JOINT_NAMES + GRIPPER_FIXED_JOINT_NAMES)
    if set(specs) != expected_joints:
        raise ValueError(f"unexpected RM65 joint set: {sorted(specs)}")
    if any(specs[name].kind != "revolute" for name in ARM_JOINT_NAMES):
        raise ValueError("all six RM65 arm joints must be revolute")
    if any(specs[name].kind != "prismatic" for name in GRIPPER_MOVABLE_JOINT_NAMES):
        raise ValueError("both gripper movable joints must be prismatic")
    if any(specs[name].kind != "fixed" for name in GRIPPER_FIXED_JOINT_NAMES):
        raise ValueError("the remaining gripper joints must be fixed")

    tcp = specs["gripper_tcp_fixed"]
    if tcp.parent != "Link6" or tcp.child != "gripper_tcp":
        raise ValueError("gripper_tcp must be attached directly to Link6")
    if not np.allclose(tcp.xyz, (0.0, 0.0, 0.170), atol=1.0e-9) or not np.allclose(tcp.rpy, 0.0):
        raise ValueError("Link6 -> gripper_tcp must be xyz=(0,0,0.170), rpy=(0,0,0)")

    mesh_paths: list[Path] = []
    text = path.read_text(encoding="utf-8")
    if "/home/li" in text or "package://" in text:
        raise ValueError("generated RM65 URDF contains a non-portable mesh path")
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename", "")
        mesh_path = Path(filename)
        if not filename or mesh_path.is_absolute() or "://" in filename:
            raise ValueError(f"RM65 mesh path must be relative: {filename!r}")
        resolved = path.parent / mesh_path
        if not resolved.is_file():
            raise FileNotFoundError(f"RM65 mesh does not resolve: {resolved}")
        mesh_paths.append(resolved)
    return {"links": tuple(links), "joints": specs, "meshes": tuple(mesh_paths)}


def fixed_mount(rp):
    """Resolve a construction-only, upright Stage 3.5 installation opt-in.

    No live model/state reference is accepted. The historical default is retained.
    """
    value = getattr(rp, "stage35_fixed_mount", None)
    if value is None:
        return FIXED_MOUNT_XYZ, FIXED_MOUNT_RPY, FIXED_MOUNT_SOURCE
    if (rp.model, rp.mount, rp.rm_control) != ("rm65", "fixed", "joint"):
        raise ValueError("Stage 3.5 installation requires rm65+fixed+joint")
    values = np.asarray(value, dtype=float)
    if values.shape != (4,) or not np.isfinite(values).all():
        raise ValueError("Stage 3.5 installation requires finite x y z yaw")
    return tuple(values[:3]), (0.0, 0.0, float(values[3])), "Stage 3.5 optional fixed fixture"


def build_fixed(builder: "newton.ModelBuilder", _rp) -> dict:
    """Add the Stage 3A RM65 with an explicit fixed world-to-base import."""
    description = inspect_urdf()
    body_start = builder.body_count
    joint_start = builder.joint_count
    xyz, rpy, source = fixed_mount(_rp)
    yaw = rpy[2]
    mount = wp.transform(
        p=wp.vec3(*xyz),
        q=wp.quat_from_axis_angle(wp.vec3(0.0, 0.0, 1.0), yaw),
    )
    builder.add_urdf(
        str(URDF_PATH),
        xform=mount,
        floating=False,
        enable_self_collisions=getattr(_rp, "rm_control", "static") == "joint",
    )

    if getattr(_rp, "rm_control", "static") == "joint":
        from .rm65_control import configure
        configure(builder)

    body_indices = {
        name: _unique_named_index(builder.body_label, name, body_start)
        for name in description["links"]
    }
    joint_indices = {
        name: _unique_named_index(builder.joint_label, name, joint_start)
        for name in description["joints"]
    }
    base = body_indices["base_link"]
    root_candidates = [
        index for index in range(joint_start, builder.joint_count)
        if int(builder.joint_parent[index]) == -1 and int(builder.joint_child[index]) == base
    ]
    if len(root_candidates) != 1:
        raise ValueError(f"expected one fixed world-to-base joint, found {len(root_candidates)}")
    fixed_base_joint = root_candidates[0]

    return {
        "model": "rm65",
        "mount": "fixed",
        "nbody": builder.body_count - body_start,
        "body_indices": body_indices,
        "joint_indices": joint_indices,
        "fixed_base_joint": fixed_base_joint,
        "fixed_base_joint_name": str(builder.joint_label[fixed_base_joint]),
        "mount_xyz": xyz,
        "mount_rpy": rpy,
        "mount_source": source,
    }


def _quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array((-q[0], -q[1], -q[2], q[3]), dtype=float)


def _quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    av, aw = a[:3], a[3]
    bv, bw = b[:3], b[3]
    return np.r_[aw * bv + bw * av + np.cross(av, bv), aw * bw - np.dot(av, bv)]


def _quat_rotate(q: np.ndarray, vector: np.ndarray) -> np.ndarray:
    return vector + 2.0 * np.cross(q[:3], np.cross(q[:3], vector) + q[3] * vector)


def _joint_type_name(value: int) -> str:
    names = {
        int(newton.JointType.FIXED): "fixed",
        int(newton.JointType.REVOLUTE): "revolute",
        int(newton.JointType.PRISMATIC): "prismatic",
    }
    return names.get(int(value), f"newton_type_{int(value)}")


def _aabb_overlap(lo: np.ndarray, hi: np.ndarray, shape_body: np.ndarray,
                  body_a: int, body_b: int) -> float:
    """Return the largest minimum-axis AABB overlap for two named bodies."""
    best = -math.inf
    for shape_a in np.where(shape_body == body_a)[0]:
        for shape_b in np.where(shape_body == body_b)[0]:
            overlap = np.minimum(hi[shape_a], hi[shape_b]) - np.maximum(lo[shape_a], lo[shape_b])
            best = max(best, float(np.min(overlap)))
    return best


def diagnose_static_scene(tm, state, contacts=None) -> dict:
    """Print and validate the name-based Stage 3.1B structure and initial state."""
    data = tm.robot_data
    if data is None or data.get("model") != "rm65" or data.get("mount") != "fixed":
        raise ValueError("RM65 static diagnostics require an rm65 + fixed scene")
    description = inspect_urdf()
    model = tm.model
    body_q = state.body_q.numpy()
    body_qd = state.body_qd.numpy()
    joint_q = model.joint_q.numpy()
    joint_qd = model.joint_qd.numpy()
    if not all(np.isfinite(values).all() for values in (body_q, body_qd, joint_q, joint_qd)):
        raise FloatingPointError("RM65 initial state contains NaN or Inf")

    body_labels = list(model.body_label)
    joint_labels = list(model.joint_label)
    body_ids = {name: int(indices[0]) for name, indices in data["body_indices"].items()}
    joint_ids = {name: int(indices[0]) for name, indices in data["joint_indices"].items()}
    for name, index in body_ids.items():
        if _short(body_labels[index]) != name:
            raise ValueError(f"body mapping mismatch for {name}")
    for name, index in joint_ids.items():
        if _short(joint_labels[index]) != name:
            raise ValueError(f"joint mapping mismatch for {name}")

    print("[rm65] bodies/links: " + ", ".join(body_ids))
    joint_type = model.joint_type.numpy()
    joint_parent = model.joint_parent.numpy()
    joint_child = model.joint_child.numpy()
    dof_start = model.joint_qd_start.numpy()
    limit_lower = model.joint_limit_lower.numpy()
    limit_upper = model.joint_limit_upper.numpy()

    def verify_and_print(name: str) -> None:
        spec = description["joints"][name]
        joint_id = joint_ids[name]
        actual_kind = _joint_type_name(joint_type[joint_id])
        actual_parent = "world" if joint_parent[joint_id] < 0 else _short(body_labels[joint_parent[joint_id]])
        actual_child = _short(body_labels[joint_child[joint_id]])
        if (actual_kind, actual_parent, actual_child) != (spec.kind, spec.parent, spec.child):
            raise ValueError(
                f"joint {name} mismatch: {(actual_kind, actual_parent, actual_child)} != "
                f"{(spec.kind, spec.parent, spec.child)}"
            )
        if spec.kind == "fixed":
            limits = "n/a"
        else:
            start = int(dof_start[joint_id])
            actual_limits = (float(limit_lower[start]), float(limit_upper[start]))
            expected_limits = (spec.lower, spec.upper)
            if not np.allclose(actual_limits, expected_limits, atol=1.0e-7):
                raise ValueError(f"joint {name} limits mismatch: {actual_limits} != {expected_limits}")
            limits = f"[{actual_limits[0]:.6g}, {actual_limits[1]:.6g}]"
        print(f"[rm65] joint {name}: type={actual_kind} parent={actual_parent} "
              f"child={actual_child} limits={limits}")

    for name in ARM_JOINT_NAMES:
        verify_and_print(name)
    for name in GRIPPER_MOVABLE_JOINT_NAMES:
        verify_and_print(name)
    for name in GRIPPER_FIXED_JOINT_NAMES:
        verify_and_print(name)

    root_joint = int(data["fixed_base_joint"][0])
    base = body_ids["base_link"]
    if (int(joint_type[root_joint]) != int(newton.JointType.FIXED)
            or int(joint_parent[root_joint]) != -1
            or int(joint_child[root_joint]) != base):
        raise ValueError("Newton world-to-base constraint is not fixed")
    print(f"[rm65] world -> base_link: fixed joint={joint_labels[root_joint]!s} "
          f"xyz={tuple(data['mount_xyz'])} rpy={tuple(data['mount_rpy'])}")
    print(f"[rm65] mount source: {data['mount_source']}")

    tcp_count = sum(_short(label) == "gripper_tcp" for label in body_labels)
    if tcp_count != tm.num_envs:
        raise ValueError(f"expected one gripper_tcp per environment, found {tcp_count}")
    link6_pose = body_q[body_ids["Link6"]]
    tcp_pose = body_q[body_ids["gripper_tcp"]]
    link6_q_inv = _quat_conjugate(link6_pose[3:])
    relative_xyz = _quat_rotate(link6_q_inv, tcp_pose[:3] - link6_pose[:3])
    relative_quat = _quat_multiply(link6_q_inv, tcp_pose[3:])
    relative_norm = float(np.linalg.norm(relative_xyz))
    if not np.allclose(relative_xyz, (0.0, 0.0, 0.170), atol=1.0e-6):
        raise ValueError(f"unexpected Link6 -> gripper_tcp translation: {relative_xyz}")
    if not np.allclose(np.abs(relative_quat[3]), 1.0, atol=1.0e-6) or not np.allclose(relative_quat[:3], 0.0, atol=1.0e-6):
        raise ValueError(f"unexpected Link6 -> gripper_tcp rotation: {relative_quat}")
    print(f"[rm65] Link6 -> gripper_tcp: xyz={relative_xyz.tolist()} "
          f"quat_xyzw={relative_quat.tolist()} norm={relative_norm:.6f} m")
    print(f"[rm65] gripper_tcp world initial pose: xyz={tcp_pose[:3].tolist()} "
          f"quat_xyzw={tcp_pose[3:].tolist()}")
    print(f"[rm65] gripper_tcp unique: yes ({tcp_count} in {tm.num_envs} environment)")

    contacts = model.contacts() if contacts is None else contacts
    model.collide(state, contacts)
    wp.synchronize()
    count = int(contacts.rigid_contact_count.numpy()[0])
    shape0 = contacts.rigid_contact_shape0.numpy()[:count]
    shape1 = contacts.rigid_contact_shape1.numpy()[:count]
    shape_body = model.shape_body.numpy()
    robot_bodies = set(body_ids.values())
    ground_contacts = environment_contacts = self_contacts = 0
    for first_shape, second_shape in zip(shape0, shape1):
        first_body = int(shape_body[int(first_shape)])
        second_body = int(shape_body[int(second_shape)])
        first_rm, second_rm = first_body in robot_bodies, second_body in robot_bodies
        if first_rm and second_rm:
            self_contacts += 1
        elif first_rm != second_rm:
            other = second_body if first_rm else first_body
            if other < 0:
                ground_contacts += 1
            else:
                environment_contacts += 1

    pipeline = model._collision_pipeline
    aabb_lo = pipeline.narrow_phase.shape_aabb_lower.numpy()
    aabb_hi = pipeline.narrow_phase.shape_aabb_upper.numpy()
    # Newton expands broad-phase AABBs by shape margin + contact gap.  Restore
    # the actual geometry bounds before using them as a penetration diagnostic.
    effective_gap = model.shape_margin.numpy() + model.shape_gap.numpy()
    geometry_lo = aabb_lo + effective_gap[:, None]
    geometry_hi = aabb_hi - effective_gap[:, None]
    robot_shapes = np.where(np.isin(shape_body, list(robot_bodies)))[0]
    minimum_z = float(np.min(geometry_lo[robot_shapes, 2]))
    left_overlap = _aabb_overlap(
        geometry_lo, geometry_hi, shape_body,
        body_ids["gripper_left_finger"], body_ids["gripper_palm"])
    right_overlap = _aabb_overlap(
        geometry_lo, geometry_hi, shape_body,
        body_ids["gripper_right_finger"], body_ids["gripper_palm"])
    if minimum_z < -0.005:
        raise ValueError(f"RM65 visibly penetrates the ground: minimum z={minimum_z:.6f}")
    if environment_contacts:
        raise ValueError(f"RM65 has {environment_contacts} initial contacts with the tree/world bodies")
    if self_contacts:
        raise ValueError(f"RM65 has {self_contacts} unexpected initial self contacts")
    if left_overlap > 0.005 or right_overlap > 0.005:
        raise ValueError(
            f"finger/palm AABB overlap is too large: left={left_overlap:.6f}, right={right_overlap:.6f}"
        )
    print(f"[rm65] initial contacts: ground={ground_contacts}, tree/environment={environment_contacts}, "
          f"self={self_contacts}; min_robot_z={minimum_z:.6f} m")
    print(f"[rm65] finger/palm AABB overlap: left={left_overlap:.6f} m, right={right_overlap:.6f} m")
    print("[rm65] finite initial state: yes")
    return {
        "body_ids": body_ids,
        "joint_ids": joint_ids,
        "tcp_world": tcp_pose.copy(),
        "tcp_relative_xyz": relative_xyz,
        "tcp_relative_quat": relative_quat,
        "tcp_relative_norm": relative_norm,
        "ground_contacts": ground_contacts,
        "environment_contacts": environment_contacts,
        "self_contacts": self_contacts,
        "minimum_z": minimum_z,
        "finger_palm_overlap": (left_overlap, right_overlap),
    }
