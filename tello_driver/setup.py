from setuptools import setup
from glob import glob
import os

package_name = "tello_driver"

setup(
    name=package_name,
    version="0.7.0",
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
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ghost",
    maintainer_email="pmbs.123@gmail.com",
    description="A simple ros package to control a DJI Tello drone",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tello_driver = tello_driver.tello_driver_node:main"
        ],
    },
)
