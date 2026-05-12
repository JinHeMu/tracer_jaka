from setuptools import setup
from glob import glob

package_name = "jaka_lerobot_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/offline", glob("offline/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="Bag-based recording + ZMQ inference bridge for Jaka + LeRobot.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bag_recorder_node = jaka_lerobot_bridge.bag_recorder_node:main",
            "policy_bridge_node = jaka_lerobot_bridge.policy_bridge_node:main",
        ],
    },
)
