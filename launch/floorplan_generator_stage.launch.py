from pathlib import Path
from shutil import copy2

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def copy_floorplan_to_bitmaps_dir(pkg_share: Path):
    bitmaps_dir = pkg_share / "world" / "bitmaps"
    bitmaps_dir.mkdir(parents=True, exist_ok=True)

    floorplan_src = pkg_share / "output" / "floorplan.png"
    floorplan_dst = bitmaps_dir / "floorplan.png"
    copy2(floorplan_src, floorplan_dst)

    print(f"Copied floorplan.png to {floorplan_dst}")


def generate_world_file_content(pkg_share: Path, config: dict) -> str:
    width = config["map"]["width"]
    height = config["map"]["height"]

    # Calculate scale to fit the map with 1 meter buffer on each side
    # Use the longest dimension to determine scale
    margin_per_side = 1.0
    longest_dimension = max(width, height)
    view_dimension = longest_dimension + 2 * margin_per_side

    # Use the smaller window dimension to ensure map fits
    window_size = min(635.0, 666.0)

    # Calculate scale: pixels per meter
    scale = window_size / view_dimension

    # Read base.world template
    template_path = pkg_share / "world" / "base.world"
    with open(template_path, "r") as f:
        template_content = f.read()

    # Replace placeholders
    world_content = template_content.replace("{{floorplan-x}}", str(width))
    world_content = world_content.replace("{{floorplan-y}}", str(height))
    world_content = world_content.replace("{{viewport-scale}}", f"{scale:.3f}")

    return world_content


def process_generated_floorplan(context):
    """Read world_config.yaml and replace placeholders in base.world template."""
    pkg_share_str = context.perform_substitution(
        FindPackageShare("floorplan_generator_stage")
    )
    pkg_share = Path(pkg_share_str)

    copy_floorplan_to_bitmaps_dir(pkg_share)

    # Read metadata YAML
    config_path = pkg_share / "output" / "world_config.yaml"
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    # Fill in the world file template
    world_content = generate_world_file_content(pkg_share, config)

    world_path = pkg_share / "world" / "generated.world"
    world_path.parent.mkdir(parents=True, exist_ok=True)
    with open(world_path, "w") as f:
        f.write(world_content)

    print(f"Generated world file: {world_path}")

    return [
        Node(
            package="stage_ros2",
            executable="stage_ros2",
            name="stage",
            output="screen",
            parameters=[
                {"world_file": str(world_path)},
                {"use_stamped_velocity": True},
            ],
        )
    ]


def generate_launch_description():
    pkg_share = FindPackageShare("floorplan_generator_stage")

    config_path_arg = DeclareLaunchArgument(
        "config_path", description="Path to the configuration file"
    )
    config_path = LaunchConfiguration("config_path")
    output_path = PathJoinSubstitution([pkg_share, "output", "floorplan.png"])

    generate_floorplan = ExecuteProcess(
        cmd=[
            "python3",
            "-m",
            "floorplan_generator.main",
            "--config",
            config_path,
            "--output",
            output_path,
        ],
        output="screen",
    )

    # Process the world template after floorplan generation completes
    process_floorplan = OpaqueFunction(function=process_generated_floorplan)

    # Register event handler to trigger template processing after generator exits
    process_floorplan_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=generate_floorplan,
            on_exit=[process_floorplan],
        )
    )

    return LaunchDescription(
        [
            config_path_arg,
            generate_floorplan,
            process_floorplan_on_exit,
        ]
    )
