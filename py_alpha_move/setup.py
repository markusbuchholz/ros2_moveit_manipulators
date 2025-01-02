from setuptools import setup
import os
from glob import glob

package_name = 'py_alpha_move'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS 2 package for controlling Reach Alpha 5 using FK/IK with PyBullet',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ik_robot_controller = py_alpha_move.ik_robot_controller:main',
            'real_ik_robot_controller = py_alpha_move.real_ik_robot_controller:main',
        ],
    },
)
