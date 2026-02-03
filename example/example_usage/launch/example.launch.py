from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tester_pkg_share = FindPackageShare("example_usage")

    # Floorplan config path for floorplan generation
    floorplan_config_path = PathJoinSubstitution(
        [tester_pkg_share, "config", "config.toml"]
    )

    # Robot config path - defines header and robot templates
    robot_config_path = PathJoinSubstitution(
        [tester_pkg_share, "config", "robot_config.yaml"]
    )

    # Path to the floorplan_generator_stage launch file
    floorplan_launch_file = PathJoinSubstitution(
        [
            FindPackageShare("floorplan_generator_stage"),
            "launch",
            "floorplan_generator_stage.launch.py",
        ]
    )

    # Launch the floorplan generator
    floorplan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(floorplan_launch_file),
        launch_arguments={
            "floorplan_config_path": floorplan_config_path,
            "robot_config_path": robot_config_path,
        }.items(),
    )

    # Stage needs STAGEPATH to find its assets (robots.inc, bitmaps, etc.)
    # All world files are consolidated in floorplan_generator_stage/world/ by the launch file
    floorplan_gen_world = PathJoinSubstitution(
        [FindPackageShare("floorplan_generator_stage"), "world"]
    )
    set_stagepath = SetEnvironmentVariable("STAGEPATH", floorplan_gen_world)

    return LaunchDescription(
        [
            set_stagepath,
            floorplan_launch,
        ]
    )
