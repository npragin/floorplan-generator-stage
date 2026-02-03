# Floorplan Generator Stage

Procedural floorplan generation and Stage simulation launcher. Generates office-style environments from configuration files and launches them in Stage.

**Clone using:** `git clone --recurse-submodules git@github.com:npragin/floorplan-generator-stage.git`

<p align="center">
  <img src="assets/example_floorplan.png" alt="Example Floorplan" width="600" height="600" />
</p>
<p align="center"><em>Example Floorplan</em></p>

## Prerequisites
- ROS 2 (tested with Jazzy and Kilted, but likely works with other distros)
- [Stage](https://github.com/rtv/Stage) and [stage_ros2](https://github.com/tuw-robotics/stage_ros2)

## Usage
I've provided an example usage as a package in `example/example_usage`. You can copy this package into your workspace and run it with `ros2 launch example_usage example.launch.py`. There are three components you're expected to provide when using this package:

### 1. Configuration Files: `example_usage/config/`

#### Floorplan Configuration: `example_usage/config/config.toml`
The floorplan configuration file contains the parameters needed to generate the floorplan. The example config contains all available parameters relevant to floorplan generation. In ambiguous cases, I've added some comments to describe what each parameter does.

**On determinism:** The seed is commented out in the provided config. When the floorplan generation backend runs, it prints the parameters it used to generate the floorplan. If you generate a floorplan you want to use later, you can copy the seed from the output.

**On experimentation:** For a faster iteration time in experimenting with parameters to see how they affect the generated floorplan, I recommend checking out the repo dedicated to the floorplan generation backend, [ros-floorplan-generator](https://github.com/npragin/ros-floorplan-generator).

#### Robot Configuration: `example_usage/config/robot_config.yaml`

The robot configuration file contains the necessary information to instantiate robots in the generated floorplan. The first field, `source_package`, is the name of the package that contains the templates I'll describe later. The second field, `header_path`, is the path to a file containing content to be inserted at the top of the generated `.world` file. The third field, `templates`, describes how to instantiate your robots in the generated floorplan. Each template has two fields: `path`, the path to the template file describing how to instantiate that robot, and `count`, the number of times to instantiate it. This provides support for heterogeneous robot types.

**On paths:** All paths in the robot configuration file are relative to the `source_package`'s `share` directory.

**On counts:** The sum of the `count` fields in the robot configuration file must be equal to the `num_robots` field in the floorplan configuration file.

### 2. Templates: `example_usage/world/`

To instantiate robots in the generated floorplan, we must paste your robot definitions into the generated `.world` file, while maintaining control over the robot's initial position. To facilitate this, you must provide a template file for each robot type you want to instantiate. The package is agnostic to the file extension of these files.

**On template location:** Templates are expected to live in the `world/` directory of the `source_package`.

#### Header: `example_usage/world/robot_header.world`

The header file is not a template. The contents of the header file will be inserted at the top of the generated `.world` file. This is where you should add any necessary includes, definitions, or other content that you want to be present in the generated `.world` file.

#### Robot Templates: `example_usage/world/pioneer2dx_with_laser_template.world`

The robot templates describe how to instantiate a robot in the generated floorplan. The template is expected to have two placeholders: `{{robot-x}}` and `{{robot-y}}`. The template will be pasted into the generated `.world` and these placeholders will be replaced with the robot's initial x and y position in the generated floorplan.

### 3. Launch File: `example_usage/launch/example.launch.py`

The floorplan generator launch file is accessible at `floorplan_generator_stage/launch/floorplan_generator_stage.launch.py`. It expects two arguments, `floorplan_config_path` and `robot_config_path`. You are responsible for managing your `STAGEPATH` environment variable, which must include the `world/` directory of the `floorplan_generator_stage` package.

**On STAGEPATH:** During development, I ran into significant pain points managing `STAGEPATH`. As a result, the package copies the entire contents of `world/` from `source_package`'s share directory to the `floorplan_generator_stage` package's `world/` in its own share directory. This means you can set your `STAGEPATH` to the `floorplan_generator_stage` package's share directory, and it will find all the necessary assets.