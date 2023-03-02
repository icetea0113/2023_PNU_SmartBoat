import glob
import os
from setuptools import setup

package_name = 'pnu2023'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob.glob("launch/*.launch.py"),
        ),
        (os.path.join("share", package_name, "param"), glob.glob("param/*.yaml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='icetea0113',
    maintainer_email='v2msyo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_convert = pnu2023.lidar_convert:main',
            'HeadingAngle = pnu2023.heading_mag:main',
            'mechaship_teleop_joystick = pnu2023.joystick:main',
            'mechaship_classify_sub_node = pnu2023.mechaship_classify_sub_node:main',
        ],
    },
)
