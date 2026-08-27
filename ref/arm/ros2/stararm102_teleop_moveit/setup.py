from glob import glob
from setuptools import find_packages, setup


PACKAGE_NAME = "stararm102_teleop_moveit"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/config", glob("config/*.yaml")),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robot-arm project",
    maintainer_email="local@example.invalid",
    description="Unified MoveIt Servo bridge for Star Arm 102-FL",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "servo_ipc_bridge = stararm102_teleop_moveit.servo_ipc_bridge:main",
        ],
    },
)
