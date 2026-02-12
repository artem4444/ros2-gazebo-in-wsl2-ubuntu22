import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pick_and_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='artem',
    maintainer_email='your_email@example.com',
    description='High-level pick and place control using ros2_control',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pick_and_place = pick_and_place.pick_and_place_node:main',
            'moveit_high_level_node = pick_and_place.moveit_high_level_node:main',
            'controller_rate_monitor_node = pick_and_place.controller_rate_monitor_node:main',
            'moveit_send_goal_example = pick_and_place.moveit_send_goal_example:main',
        ],
    },
)
