from setuptools import find_packages, setup

package_name = "slam_bot_bridge"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SLAM Bot",
    maintainer_email="you@example.com",
    description="MCU <-> ROS2 bridge and frontend relay for the SLAM Bot.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Standalone bridge: connects to the FastAPI backend as a WebSocket
            # client and republishes telemetry onto ROS topics. Only needed if
            # you run the bridge separately from the backend process.
            "bridge_node = slam_bot_bridge.bridge_node:main",
            # Forwards ROS topics to the backend so the browser can render them.
            "web_relay = slam_bot_bridge.web_relay:main",
        ],
    },
)
