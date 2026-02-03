from glob import glob

from setuptools import find_packages, setup

package_name = "floorplan_generator_stage"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]) + ["floorplan_generator"],
    package_dir={
        "floorplan_generator": (
            "floorplan_generator_stage/floorplan_generator/floorplan_generator"
        ),
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            glob("launch/*.launch.py"),
        ),
        (
            "share/" + package_name + "/world",
            glob("world/*.world"),
        ),
    ],
    package_data={"": ["py.typed"]},
    install_requires=[
        "setuptools",
        "shapely>=2.0.0",
        "numpy>=1.26.0",
        "pillow>=10.0.0",
        "typer>=0.9.0",
        "pyyaml>=6.0",
    ],
    zip_safe=True,
    maintainer="npragin",
    maintainer_email="npragin@gmail.com",
    description="Procedural floorplan generation and Stage simulation launcher. Generates office-style environments "
    "from TOML configurations and launches them in Stage.",
    license="GNU General Public License v3.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [],
    },
)
