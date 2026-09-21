"""Focused Stage 3.1B regression tests; no window or harvesting is started."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import unittest
from unittest import mock

import numpy as np

from scripts import grow_tree
from treesim import builder, rm65, robot
from treesim.config import RobotParams, TreeConfig
from treesim.sim import Sim


REPO_ROOT = Path(__file__).resolve().parents[1]
ARM_SOURCE = Path("/home/li/ros2_ws/src/ros2_rm_robot/rm_description/urdf/rm_65.urdf")
GRIPPER_SOURCE = Path("/home/li/ros2_ws/src/rm_rl_grasp/urdf/rm65_with_gripper.urdf.xacro")


def rm_scene(*, gravity: float = -9.81):
    config = TreeConfig.rigid("apple")
    config.lsystem.n = 2
    config.seed = 31
    config.device = "cuda"
    config.physics.gravity = gravity
    config.robot.enabled = True
    config.robot.model = "rm65"
    config.robot.mount = "fixed"
    config.robot.camera = False
    return builder.generate_and_build(config, num_envs=1)


class Stage31BSelectionTests(unittest.TestCase):
    def test_new_parameter_defaults_preserve_baseline_selection(self):
        args = grow_tree.parse_args([])
        self.assertEqual((args.robot_model, args.mount), ("franka", "ridgeback"))
        config = grow_tree.make_config(args)
        self.assertEqual((config.robot.model, config.robot.mount), ("franka", "ridgeback"))
        self.assertFalse(config.robot.enabled)

    def test_rm65_fixed_selection_implies_robot_without_camera(self):
        args = grow_tree.parse_args(["--robot-model", "rm65", "--mount", "fixed"])
        config = grow_tree.make_config(args)
        self.assertTrue(config.robot.enabled)
        self.assertFalse(config.robot.camera)

    def test_rm65_ridgeback_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "unsupported"):
            robot.validate_robot_configuration("rm65", "ridgeback")

    def test_franka_fixed_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "unsupported"):
            robot.validate_robot_configuration("franka", "fixed")

    def test_rm65_auto_is_rejected_before_build(self):
        args = grow_tree.parse_args([
            "--robot-model", "rm65", "--mount", "fixed", "--auto"
        ])
        with self.assertRaisesRegex(ValueError, "unavailable"):
            grow_tree.make_config(args)

    def test_franka_dispatch_still_uses_original_builder(self):
        params = RobotParams(enabled=True, model="franka", mount="ridgeback")
        marker = {"baseline": True, "model": "franka", "mount": "ridgeback"}
        with mock.patch.object(robot, "_build_ridgeback_franka", return_value=marker) as baseline:
            result = robot.build_robot(object(), params)
        baseline.assert_called_once()
        self.assertTrue(result["baseline"])
        self.assertEqual((result["model"], result["mount"]), ("franka", "ridgeback"))


class Stage31BURDFTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.description = rm65.inspect_urdf()

    def test_key_links_exist_by_name(self):
        self.assertEqual(set(rm65.REQUIRED_LINK_NAMES), set(self.description["links"]))

    def test_six_arm_joints(self):
        joints = self.description["joints"]
        self.assertEqual(6, sum(joints[name].kind == "revolute" for name in rm65.ARM_JOINT_NAMES))

    def test_two_movable_gripper_joints(self):
        joints = self.description["joints"]
        self.assertEqual(2, sum(joints[name].kind == "prismatic"
                                for name in rm65.GRIPPER_MOVABLE_JOINT_NAMES))

    def test_gripper_tcp_is_unique(self):
        self.assertEqual(1, self.description["links"].count("gripper_tcp"))

    def test_link6_to_tcp_is_170_mm(self):
        tcp = self.description["joints"]["gripper_tcp_fixed"]
        self.assertEqual((tcp.parent, tcp.child), ("Link6", "gripper_tcp"))
        self.assertAlmostEqual(np.linalg.norm(tcp.xyz), 0.170, places=7)
        np.testing.assert_allclose(tcp.rpy, 0.0, atol=1.0e-9)

    def test_mesh_paths_are_portable_and_resolve(self):
        text = rm65.URDF_PATH.read_text(encoding="utf-8")
        self.assertNotIn("/home/li", text)
        self.assertNotIn("package://", text)
        self.assertTrue(self.description["meshes"])
        self.assertTrue(all(path.is_file() for path in self.description["meshes"]))

    def test_stage3a_generator_check(self):
        command = [
            sys.executable,
            str(REPO_ROOT / "tools" / "generate_rm65_newton_urdf.py"),
            "--arm-urdf", str(ARM_SOURCE),
            "--gripper-source", str(GRIPPER_SOURCE),
            "--check",
        ]
        result = subprocess.run(command, cwd=REPO_ROOT, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("synchronized", result.stdout)


class Stage31BSceneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tree = rm_scene()

    def test_rm65_fixed_builds_in_tree_world(self):
        self.assertEqual((self.tree.robot_data["model"], self.tree.robot_data["mount"]),
                         ("rm65", "fixed"))
        self.assertGreater(self.tree.n_bodies, 0)

    def test_actual_links_and_joints_resolve_by_name(self):
        self.assertEqual(set(rm65.REQUIRED_LINK_NAMES), set(self.tree.robot_data["body_indices"]))
        expected_joints = set(
            rm65.ARM_JOINT_NAMES
            + rm65.GRIPPER_MOVABLE_JOINT_NAMES
            + rm65.GRIPPER_FIXED_JOINT_NAMES
        )
        self.assertEqual(expected_joints, set(self.tree.robot_data["joint_indices"]))

    def test_fixed_world_to_base_constraint_exists(self):
        model = self.tree.model
        root = int(self.tree.robot_data["fixed_base_joint"][0])
        base = int(self.tree.robot_data["body_indices"]["base_link"][0])
        self.assertEqual(int(model.joint_type.numpy()[root]), int(robot.newton.JointType.FIXED))
        self.assertEqual(int(model.joint_parent.numpy()[root]), -1)
        self.assertEqual(int(model.joint_child.numpy()[root]), base)

    def test_initial_scene_diagnostics(self):
        state, _ = self.tree.state_pair()
        report = rm65.diagnose_static_scene(self.tree, state)
        self.assertEqual(report["environment_contacts"], 0)
        self.assertEqual(report["self_contacts"], 0)
        self.assertAlmostEqual(report["tcp_relative_norm"], 0.170, places=6)

    def test_isolated_zero_gravity_short_stability_fixture(self):
        tree = rm_scene(gravity=0.0)
        sim = Sim(tree, solver="mujoco", fps=60, substeps=1, collisions=True)
        base = int(tree.robot_data["body_indices"]["base_link"][0])
        initial_base = sim.state_0.body_q.numpy()[base].copy()
        for _ in range(12):
            sim.step()
        body_q = sim.state_0.body_q.numpy()
        body_qd = sim.state_0.body_qd.numpy()
        joint_q = sim.state_0.joint_q.numpy()
        self.assertTrue(np.isfinite(body_q).all())
        self.assertTrue(np.isfinite(body_qd).all())
        self.assertTrue(np.isfinite(joint_q).all())
        np.testing.assert_allclose(body_q[base], initial_base, atol=1.0e-6)
        self.assertLess(float(np.max(np.abs(body_qd))), 1.0e-3)


if __name__ == "__main__":
    unittest.main()
