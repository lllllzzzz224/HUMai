"""Recorded-state display and MoveIt planning only, isolated on localhost domain 87."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    if (os.environ.get('ROS_DOMAIN_ID') != '87'
            or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'
            or os.environ.get('ROS_STATIC_PEERS')):
        raise RuntimeError('Use ROS_DOMAIN_ID=87 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST without static peers')
    urdf = str(Path(get_package_share_directory('rm_description'))/'urdf/rm_65.urdf')
    c = (MoveItConfigsBuilder('rm_65_description', package_name='rm_65_config')
         .robot_description(file_path=urdf)
         .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
         .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
         .to_moveit_configs())
    c.planning_pipelines['ompl']['rm_group'] = {
        'planner_configs':['RRTConnect'], 'default_planner_config':'RRTConnect',
        'projection_evaluator':'joints(joint1,joint2)', 'longest_valid_segment_fraction':.005}
    model = {**c.robot_description, **c.robot_description_semantic,
             **c.robot_description_kinematics, **c.joint_limits, **c.planning_pipelines}
    gate = {**c.planning_scene_monitor, 'allow_trajectory_execution':False,
            'moveit_manage_controllers':False, 'monitor_dynamics':False, 'use_sim_time':False,
            'disable_capabilities': 'move_group/MoveGroupExecuteTrajectoryAction move_group/MoveGroupExecuteService'}
    remaps = [('/joint_states', '/offline_preview/joint_states')]
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[c.robot_description], remappings=remaps, output='screen'),
        Node(package='moveit_ros_move_group', executable='move_group',
             parameters=[model, gate], remappings=remaps, output='screen'),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(Path(__file__).with_name('obstacle_preview.rviz'))],
             parameters=[model], remappings=remaps, output='screen'),
    ])
