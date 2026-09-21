# OrchardBench Repository Instructions

1. Work must proceed in explicit stages. Do not begin a later stage without the user's permission.
2. Do not modify the OrchardBench tree, apple, stem, detachment, collision, or related physical parameters unless the user explicitly changes this rule.
3. Preserve the original Franka baseline and its default behavior. RM65 integration must remain optional and switchable rather than replacing or overwriting Franka.
4. RM65 model sources are fixed as follows:
   - Arm motion chain, links, joints, limits, inertias, visuals, and collisions: the official ordinary non-Gazebo RM65 URDF at `/home/li/ros2_ws/src/ros2_rm_robot/rm_description/urdf/rm_65.urdf`.
   - Gripper links, joints, geometry, collisions, and limits: the user-authored definitions in `/home/li/ros2_ws/src/rm_rl_grasp/urdf/rm65_with_gripper.urdf.xacro`.
   Do not substitute a Gazebo-specific URDF, reverse-expand the combined Xacro, or redesign the gripper geometry without explicit permission.
5. Every modification must be validated by an actual, proportionate check or run. Report the validation performed and its result.
6. Do not create Git commits unless the user explicitly requests a commit. Do not stage files, alter remotes, or rewrite Git history without explicit permission.
