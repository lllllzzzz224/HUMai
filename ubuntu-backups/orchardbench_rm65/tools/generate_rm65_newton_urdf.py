#!/usr/bin/env python3
"""Build OrchardBench's plain Newton RM65 URDF from its two authoritative sources.

The arm links, joints, limits, inertias, visuals, and collisions come from the
official non-Gazebo RM65 URDF.  Only the explicitly listed gripper links and
joints are copied from the locally authored gripper Xacro; no Xacro expansion
is performed.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = REPO_ROOT / "assets" / "robots" / "rm65"

ARM_TOOL_LINKS = {"gripper_body_approx", "tcp_link"}
ARM_TOOL_JOINTS = {"link6_to_gripper_body_approx", "link6_to_tcp"}
GRIPPER_LINKS = (
    "gripper_mount",
    "gripper_palm",
    "gripper_left_finger",
    "gripper_right_finger",
    "gripper_tcp",
)
GRIPPER_JOINTS = (
    "gripper_mount_fixed",
    "gripper_palm_fixed",
    "gripper_left_joint",
    "gripper_right_joint",
    "gripper_tcp_fixed",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def named_child(root: ET.Element, tag: str, name: str) -> ET.Element:
    matches = [item for item in root.findall(tag) if item.get("name") == name]
    if len(matches) != 1:
        raise ValueError(f"expected exactly one {tag} named {name!r}, found {len(matches)}")
    return matches[0]


def remove_named(root: ET.Element, tag: str, names: set[str]) -> set[str]:
    found: set[str] = set()
    for item in list(root.findall(tag)):
        name = item.get("name")
        if name in names:
            root.remove(item)
            found.add(name)
    return found


def source_mesh(filename: str, arm_urdf: Path) -> tuple[Path, Path]:
    description_root = arm_urdf.resolve().parent.parent
    prefix = "package://rm_description/"
    if filename.startswith(prefix):
        relative = Path(filename[len(prefix) :])
        return description_root / relative, relative

    path = Path(filename)
    resolved = path if path.is_absolute() else arm_urdf.resolve().parent / path
    try:
        relative = resolved.resolve().relative_to(description_root)
    except ValueError as exc:
        raise ValueError(f"mesh is outside rm_description and cannot be vendored safely: {filename}") from exc
    return resolved, relative


def build_tree(arm_urdf: Path, gripper_source: Path) -> tuple[ET.ElementTree, list[tuple[Path, Path]]]:
    arm_tree = ET.parse(arm_urdf)
    arm_root = arm_tree.getroot()
    if arm_root.tag != "robot":
        raise ValueError("arm source is not a plain URDF <robot>")
    if any("xacro" in element.tag or element.tag.endswith("gazebo") or element.tag.endswith("ros2_control")
           for element in arm_root.iter()):
        raise ValueError("arm source contains Xacro, Gazebo, or ros2_control elements")

    removed_links = remove_named(arm_root, "link", ARM_TOOL_LINKS)
    removed_joints = remove_named(arm_root, "joint", ARM_TOOL_JOINTS)
    removed_count = len(removed_links) + len(removed_joints)
    if removed_count not in (0, len(ARM_TOOL_LINKS) + len(ARM_TOOL_JOINTS)):
        raise ValueError("arm source contains only part of the optional legacy tool definition")

    gripper_root = ET.parse(gripper_source).getroot()
    for name in GRIPPER_LINKS:
        arm_root.append(copy.deepcopy(named_child(gripper_root, "link", name)))
    for name in GRIPPER_JOINTS:
        arm_root.append(copy.deepcopy(named_child(gripper_root, "joint", name)))

    tcp_joint = named_child(arm_root, "joint", "gripper_tcp_fixed")
    parent = tcp_joint.find("parent")
    child = tcp_joint.find("child")
    origin = tcp_joint.find("origin")
    if parent is None or parent.get("link") != "Link6":
        raise ValueError("gripper_tcp_fixed must be parented directly to Link6")
    if child is None or child.get("link") != "gripper_tcp":
        raise ValueError("gripper_tcp_fixed must end at gripper_tcp")
    if origin is None or [float(value) for value in origin.get("xyz", "").split()] != [0.0, 0.0, 0.170]:
        raise ValueError("Link6 to gripper_tcp must be exactly 0 0 0.170 m")

    arm_root.set("name", "rm65_with_gripper_newton")
    meshes: list[tuple[Path, Path]] = []
    for mesh in arm_root.findall(".//mesh"):
        filename = mesh.get("filename")
        if not filename:
            raise ValueError("mesh element has no filename")
        source, relative = source_mesh(filename, arm_urdf)
        if not source.is_file():
            raise FileNotFoundError(f"missing source mesh: {source}")
        destination = relative if relative.parts and relative.parts[0] == "meshes" else Path("meshes") / relative
        mesh.set("filename", destination.as_posix())
        meshes.append((source, destination))

    return arm_tree, meshes


def render(tree: ET.ElementTree, arm_urdf: Path, gripper_source: Path) -> bytes:
    ET.indent(tree, space="  ")
    body = ET.tostring(tree.getroot(), encoding="unicode", short_empty_elements=True)
    header = (
        "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
        "<!-- GENERATED FILE: do not edit by hand.\n"
        "     Generator: tools/generate_rm65_newton_urdf.py\n"
        f"     Arm source SHA256: {sha256(arm_urdf)}\n"
        f"     Gripper source SHA256: {sha256(gripper_source)}\n"
        "     See assets/robots/rm65/README.md for regeneration. -->\n"
    )
    return (header + body + "\n").encode("utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm-urdf", required=True, type=Path)
    parser.add_argument("--gripper-source", required=True, type=Path)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--check", action="store_true", help="verify generated files without writing")
    args = parser.parse_args()

    arm_urdf = args.arm_urdf.resolve()
    gripper_source = args.gripper_source.resolve()
    output_dir = args.output_dir.resolve()
    tree, meshes = build_tree(arm_urdf, gripper_source)
    expected_urdf = render(tree, arm_urdf, gripper_source)
    output_urdf = output_dir / "rm65_with_gripper.urdf"

    if args.check:
        errors: list[str] = []
        if not output_urdf.is_file() or output_urdf.read_bytes() != expected_urdf:
            errors.append(f"out of date: {output_urdf}")
        for source, relative in meshes:
            destination = output_dir / relative
            if not destination.is_file() or sha256(destination) != sha256(source):
                errors.append(f"missing or out of date: {destination}")
        if errors:
            print("\n".join(errors), file=sys.stderr)
            return 1
        print(f"RM65 Newton assets are synchronized: {output_urdf}")
        return 0

    output_dir.mkdir(parents=True, exist_ok=True)
    output_urdf.write_bytes(expected_urdf)
    for source, relative in meshes:
        destination = output_dir / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
    print(f"generated {output_urdf} with {len(meshes)} referenced mesh instances")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
