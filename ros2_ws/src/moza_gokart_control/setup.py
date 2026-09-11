from glob import glob
import os

from setuptools import find_packages, setup


package_name = "moza_gokart_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="KAR AI Copilot Team",
    maintainer_email="maintainer@example.com",
    description="Fail-safe MOZA R5 manual control pipeline for the KAR GoKart.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "moza_input_node = moza_gokart_control.moza_input_node:main",
            "shared_control_node = moza_gokart_control.shared_control_node:main",
            "isaac_adapter_node = moza_gokart_control.isaac_adapter_node:main",
            "dynamics_logger_node = moza_gokart_control.dynamics_logger_node:main",
        ],
    },
)
