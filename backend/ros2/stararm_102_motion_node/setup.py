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
            [
                "config/controllers.yaml",
                "config/servo.yaml",
                "config/sensors_3d.yaml",
                "config/stararm102.rviz",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
)
