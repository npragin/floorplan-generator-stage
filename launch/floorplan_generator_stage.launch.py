from pathlib import Path
from shutil import copy2, copytree

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def copy_floorplan_to_bitmaps_dir(pkg_share: Path):
    bitmaps_dir = pkg_share / "world" / "bitmaps"
    bitmaps_dir.mkdir(parents=True, exist_ok=True)

    floorplan_src = pkg_share / "output" / "floorplan.png"
    floorplan_dst = bitmaps_dir / "floorplan.png"
    copy2(floorplan_src, floorplan_dst)

    print(f"Copied floorplan.png to {floorplan_dst}")


def copy_world_files_from_source_package(
    pkg_share: Path, source_package: str | None
) -> None:
    """Copy world directory from source package to floorplan_generator_stage."""
    if not source_package:
        raise ValueError(
            "No source_package specified in robot_config.yaml. "
            "Please add 'source_package: <package_name>' to your robot configuration."
        )

    try:
        source_pkg_share = Path(get_package_share_directory(source_package))
    except Exception as e:
        raise RuntimeError(
            f"Could not find package '{source_package}' provided in robot configuration file. "
            f"Ensure the package is installed and sourced. Error: {e}"
        ) from e

    source_world_dir = source_pkg_share / "world"
    if not source_world_dir.exists():
        raise RuntimeError(
            f"No world/ directory found in package '{source_package}'. "
            f"Expected directory at: {source_world_dir}"
        )

    dest_world_dir = pkg_share / "world"
    copytree(source_world_dir, dest_world_dir, dirs_exist_ok=True)

    print(f"Copied world/ directory from {source_package} to {dest_world_dir}")


def load_robot_config(
    pkg_share: Path, robot_config_path_str: str | None
) -> tuple[str | None, dict[str, int]]:
    """Load robot config from YAML, copy world files, and resolve paths.

    Returns:
        Tuple of (robot_header_path, robot_templates).
    """
    robot_header_path = None
    robot_templates: dict[str, int] = {}
    source_package = None

    if not robot_config_path_str:
        return robot_header_path, robot_templates

    with open(robot_config_path_str, "r") as f:
        robot_config = yaml.safe_load(f)

    # Extract source package and copy its world directory
    source_package = robot_config.get("source_package")
    copy_world_files_from_source_package(pkg_share, source_package)

    if source_package:
        header_path_rel = robot_config.get("header_path")
        if header_path_rel:
            robot_header_path = str(pkg_share / header_path_rel)

        # Convert template list to dict
        for template in robot_config.get("templates", []):
            template_path_rel = template["path"]
            template_path_abs = str(pkg_share / template_path_rel)
            robot_templates[template_path_abs] = template["count"]

    return robot_header_path, robot_templates


def generate_robot_instances(
    robot_templates: dict[str, int], spawn_positions: list[dict]
) -> str:
    """Generate robot instances from templates and spawn positions."""
    total_robots_requested = sum(robot_templates.values())
    num_spawn_positions = len(spawn_positions)

    if total_robots_requested != num_spawn_positions:
        raise ValueError(
            f"Robot count mismatch: {total_robots_requested} robots requested in template "
            f"dictionary, but {num_spawn_positions} spawn positions available in world_config.yaml. "
            f"Either adjust the robot template dictionary quantities or change the 'num_robots' "
            f"parameter in the floorplan generation config TOML."
        )

    robot_instances = []
    position_index = 0

    for template_path, count in robot_templates.items():
        with open(template_path, "r") as f:
            template_content = f.read()

        for _ in range(count):
            spawn = spawn_positions[position_index]
            x = spawn["x"]
            y = spawn["y"]
            instance = template_content.replace("{{robot-x}}", str(x))
            instance = instance.replace("{{robot-y}}", str(y))
            robot_instances.append(instance)
            position_index += 1

    return "\n\n".join(robot_instances)


def generate_world_file_content(
    pkg_share: Path,
    config: dict,
    robot_header_path: str | None,
    robot_templates: dict[str, int],
) -> str:
    width = config["map"]["width"]
    height = config["map"]["height"]

    # Add 1 meter buffer on each side of the longest dimension
    margin_per_side = 1.0
    longest_dimension = max(width, height)
    view_dimension = longest_dimension + 2 * margin_per_side

    # Use the smaller window dimension to ensure map fits
    # Numbers come from example stage world files. If changing, update base.world template.
    window_size = min(635.0, 666.0)
    scale = window_size / view_dimension

    header_content = ""
    if robot_header_path:
        with open(robot_header_path, "r") as f:
            header_content = f.read().strip() + "\n\n"

    # Replace placeholders in base.world template
    template_path = pkg_share / "world" / "base.world"
    with open(template_path, "r") as f:
        template_content = f.read()
    base_content = template_content.replace("{{floorplan-x}}", str(width))
    base_content = base_content.replace("{{floorplan-y}}", str(height))
    base_content = base_content.replace("{{viewport-scale}}", f"{scale:.3f}")

    robot_content = ""
    if robot_templates:
        spawn_positions = config.get("robots", [])
        robot_content = "\n\n" + generate_robot_instances(
            robot_templates, spawn_positions
        )

    world_content = header_content + base_content + robot_content

    return world_content


def process_generated_floorplan(context):
    """Read world_config.yaml and replace placeholders in base.world template."""
    pkg_share_str = context.perform_substitution(
        FindPackageShare("floorplan_generator_stage")
    )
    pkg_share = Path(pkg_share_str)

    copy_floorplan_to_bitmaps_dir(pkg_share)

    config_path = pkg_share / "output" / "world_config.yaml"
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    # Get robot config path and parse it
    robot_config_path_str = context.perform_substitution(
        LaunchConfiguration("robot_config_path")
    )
    robot_header_path, robot_templates = load_robot_config(
        pkg_share, robot_config_path_str
    )

    # Fill in the world file template
    world_content = generate_world_file_content(
        pkg_share, config, robot_header_path, robot_templates
    )

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

    floorplan_config_path_arg = DeclareLaunchArgument(
        "floorplan_config_path",
        description="Path to the floorplan generation configuration file",
    )
    floorplan_config_path = LaunchConfiguration("floorplan_config_path")

    robot_config_path_arg = DeclareLaunchArgument(
        "robot_config_path",
        default_value="",
        description="Path to robot configuration YAML file (header_path and templates)",
    )

    output_path = PathJoinSubstitution([pkg_share, "output", "floorplan.png"])

    generate_floorplan = ExecuteProcess(
        cmd=[
            "python3",
            "-m",
            "floorplan_generator.main",
            "--config",
            floorplan_config_path,
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
            floorplan_config_path_arg,
            robot_config_path_arg,
            generate_floorplan,
            process_floorplan_on_exit,
        ]
    )
