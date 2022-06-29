from setuptools import setup
import subprocess, os, platform
from glob import glob

package_name = "moveit_py_example"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "launch"), glob("launch/rviz/*.rviz")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Peter David Fagan",
    author_email="peterdavidfagan@gmail.com",
    maintainer="Peter David Fagan",
    maintainer_email="peterdavidfagan@gmail.com",
    description="package description",
    license="license",
    entry_points={
        "console_scripts": [
            "pose_goal = moveit_py_example.pose_goal:main",
            "planning_scene_interface = moveit_py_example.planning_scene_interface:main"
            ],
        }
        )
