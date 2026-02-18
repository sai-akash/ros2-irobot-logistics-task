#!/usr/bin/env python
import os
from setuptools import setup

package_name = 'rof_flexbe_behaviors'

setup(
    name=package_name,
    version='1.3.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Julian',
    maintainer_email='julian.sessner@faps.fau.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_behavior_sm = rof_flexbe_behaviors.example_behavior_sm',
            'move_jetbot = rof_flexbe_behaviors.move_jetbot_sm',
        ],
    },
)
