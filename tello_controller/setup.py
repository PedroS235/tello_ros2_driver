from setuptools import setup
from glob import glob
import os

package_name = "tello_controller"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    # install_requires=['setuptools'],
    zip_safe=True,
    maintainer="Pedro Soares",
    maintainer_email="pmbs.123@gmail.com",
    description="Simple keyboard controller for Tello",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tello_controller = tello_controller.tello_controller_node:main"
        ],
    },
)
