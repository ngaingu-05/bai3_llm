import os
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, relative_path):
    file_path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(file_path, encoding="utf-8") as config_file:
        return yaml.safe_load(config_file)


def launch_moveit(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type").perform(context)
    moveit_share = get_package_share_directory("ur_moveit_config")
    skills_share = get_package_share_directory("ur3_llm_robot_skills")
    semantic_path = Path(skills_share) / "urdf" / "ur3_parallel_gripper.srdf.xacro"
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(semantic_path, {"name": "ur"})
        .to_moveit_configs()
    )
    warehouse = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
    }
    wait_for_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            warehouse,
            {"use_sim_time": True, "publish_robot_description_semantic": True},
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", os.path.join(moveit_share, "config", "moveit.rviz")],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse,
            {"use_sim_time": True},
        ],
    )
    return [
        wait_for_description,
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_for_description,
                on_exit=[move_group, rviz],
            )
        ),
    ]


def generate_launch_description():
    simulation_share = get_package_share_directory("ur_simulation_gz")
    moveit_share = get_package_share_directory("ur_moveit_config")
    skills_share = get_package_share_directory("ur3_llm_robot_skills")
    kinematics = load_yaml("ur_moveit_config", "config/kinematics.yaml")
    scene = load_yaml("ur3_llm_robot_skills", "config/scene.yaml")
    scene_parameters = scene["ur3_skill_executor"]["ros__parameters"]
    ur_type = LaunchConfiguration("ur_type")

    robot_description = {
        "robot_description": Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([
                FindPackageShare("ur3_llm_robot_skills"), "urdf", "ur3_parallel_gripper.urdf.xacro"
            ]), " ",
            "safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20 ",
            "name:=ur ur_type:=", ur_type, " tf_prefix:=\"\" ",
            "simulation_controllers:=", PathJoinSubstitution([
                FindPackageShare("ur3_llm_robot_skills"), "config", "controllers.yaml"
            ]),
        ])
    }
    robot_description_semantic = {
        "robot_description_semantic": Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([
                FindPackageShare("ur3_llm_robot_skills"), "urdf", "ur3_parallel_gripper.srdf.xacro"
            ]), " name:=ur",
        ])
    }

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(simulation_share, "launch", "ur_sim_control.launch.py")),
        launch_arguments={
            "ur_type": ur_type,
            "gazebo_gui": LaunchConfiguration("gazebo_gui"),
            "description_file": os.path.join(skills_share, "urdf", "ur3_parallel_gripper.urdf.xacro"),
            "controllers_file": os.path.join(skills_share, "config", "controllers.yaml"),
            "world_file": os.path.join(skills_share, "worlds", "ur3_pick_place.sdf"),
            "initial_joint_controller": "scaled_joint_trajectory_controller",
            "launch_rviz": "false",
        }.items(),
    )

    # ── Spawn gripper_controller after Gazebo is settled (10 s wait).
    # Pass --controller-manager-timeout 60 so the spawner keeps retrying for
    # up to 60 s instead of the default 10 s × 3 attempts.
    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )

    # ── Start skill executor only AFTER gripper_controller spawner exits
    # successfully (i.e. the gripper action server is guaranteed to be live).
    skills = Node(
        package="ur3_llm_robot_skills",
        executable="robot_skills",
        name="ur3_skill_executor",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics, "use_sim_time": True},
            scene_parameters,
        ],
    )

    planner = Node(
        package="ur3_llm_control",
        executable="llm_planner",
        name="ur3_llm_planner",
        output="screen",
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/overhead_cam@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        output="screen",
    )

    camera_detector = Node(
        package="ur3_llm_control",
        executable="camera_detector",
        name="camera_detector",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "ur_type", default_value="ur3e", choices=["ur3", "ur3e"],
            description="UR model used by the simulator and MoveIt.",
        ),
        DeclareLaunchArgument("gazebo_gui", default_value="true", description="Start Gazebo with its GUI."),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        simulation,
        OpaqueFunction(function=launch_moveit),
        camera_bridge,
        camera_detector,
        planner,
        # Wait 10 s for Gazebo + controller_manager to be fully ready,
        # then spawn the gripper controller.
        TimerAction(period=10.0, actions=[gripper_controller]),
        # Only start the skill executor after gripper spawner completes.
        RegisterEventHandler(
            OnProcessExit(
                target_action=gripper_controller,
                on_exit=[skills],
            )
        ),
    ])