from setuptools import setup

package_name = 'py_alpha_move'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='A simple ROS 2 package that prints Hello, ROS 2!',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_node = py_alpha_move.hello:main',
            'ik_robot_controller = py_alpha_move.ik_robot_controller:main',
        ],
    },
)
