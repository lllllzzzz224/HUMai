#!/usr/bin/env python3
"""Publish the calibrated Link6-to-camera_link static transform."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="hand_eye_static_tf",
            output="screen",
            arguments=[
                "-0.064514447085", "0.005428183272", "0.003696935131",
                "-0.020548815920", "0.728253776074", "-0.001728512288",
                "-0.684997223383", "Link6", "camera_link",
            ],
        )
    ])
