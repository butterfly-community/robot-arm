from setuptools import find_packages, setup

package_name = "stararm_102_motion_node"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/motion_stack.launch.py"]),
        (
            "share/" + package_name + "/config",
            ["config/controllers.yaml", "config/servo.yaml"],
        ),
    ],
    install_requires=["setuptools", "transforms3d==0.4.2"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "dora_motion_node = stararm_102_motion_node.dora_motion_node:main"
        ]
    },
)
