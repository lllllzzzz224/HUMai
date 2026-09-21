#!/usr/bin/env python3
"""One-command bringup for the visual hand-eye consistency test.

Starts the physical RM65 stack, RealSense, the calibrated hand-eye bridge,
the saved RViz layout, and the vision-only apple centre estimator. MoveIt
trajectory execution is disabled for this test launch.
"""

from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from project_paths import (
    APPLE_PICK_DIR,
    REPO_ROOT,
    default_model_path,
    default_yolo_python,
)

PROJECT = APPLE_PICK_DIR
RVIZ_CONFIG = PROJECT / "apple_hand_eye.rviz"
APPLE_NODE = PROJECT / "apple_center_localizer_v2.py"
HAND_EYE_LAUNCH = REPO_ROOT / "hand_eye_static_tf.launch.py"


def package_launch(package: str, launch_file: str) -> PythonLaunchDescriptionSource:
    return PythonLaunchDescriptionSource(
        str(Path(get_package_share_directory(package)) / "launch" / launch_file)
    )


def generate_launch_description() -> LaunchDescription:
    required = [RVIZ_CONFIG, APPLE_NODE, HAND_EYE_LAUNCH]
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise RuntimeError(f"Required apple hand-eye files are missing: {missing}")

    # Start rm_control before MoveIt. Starting both simultaneously can leave
    # MoveIt with a disconnected controller handle for its whole lifetime.
    rm_driver = IncludeLaunchDescription(
        package_launch("rm_driver", "rm_65_driver.launch.py")
    )
    rm_description = IncludeLaunchDescription(
        package_launch("rm_description", "rm_65_display.launch.py")
    )
    rm_control = IncludeLaunchDescription(
        package_launch("rm_control", "rm_65_control.launch.py")
    )
    moveit = TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(
            package_launch("rm_65_config", "real_moveit_demo.launch.py"),
            launch_arguments={
                "rviz_config": str(RVIZ_CONFIG),
                "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
            }.items(),
        )],
    )

    realsense = IncludeLaunchDescription(
        package_launch("realsense2_camera", "rs_launch.py"),
        launch_arguments={
            "enable_color": "true",
            "enable_depth": "true",
            "align_depth.enable": "true",
            "enable_sync": "true",
            "pointcloud.enable": "true",
            "publish_tf": "true",
        }.items(),
    )

    hand_eye = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(HAND_EYE_LAUNCH))
    )

    # Give the robot TF and camera streams time to appear before loading YOLO.
    apple_localizer = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    LaunchConfiguration("yolo_python"),
                    str(APPLE_NODE),
                    "--model",
                    LaunchConfiguration("yolo_model"),
                    "--inference-hz",
                    LaunchConfiguration("inference_hz"),
                    "--publish-candidates",
                    LaunchConfiguration("publish_candidates"),
                    "--target",
                    LaunchConfiguration("target"),
                ],
                name="apple_center_localizer_v2_process",
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "allow_trajectory_execution",
            default_value="false",
            description="Keep false for hand-eye tests; set true only for guarded motion.",
        ),
        DeclareLaunchArgument(
            "yolo_python",
            default_value=str(default_yolo_python()),
            description="Python executable containing ultralytics.",
        ),
        DeclareLaunchArgument(
            "yolo_model",
            default_value=str(default_model_path()),
            description="YOLO segmentation model path.",
        ),
        DeclareLaunchArgument("inference_hz", default_value="2.0",
                              description="Maximum perception rate; hardware throughput may be lower."),
        DeclareLaunchArgument("publish_candidates", default_value="false",
                              description="Opt-in complete timestamped multi-fruit snapshots."),
        DeclareLaunchArgument(
            "target",
            default_value="orange",
            description="Target fruit to localize: apple, orange, or all.",
        ),
        rm_driver,
        rm_description,
        rm_control,
        moveit,
        realsense,
        hand_eye,
        apple_localizer,
    ])
