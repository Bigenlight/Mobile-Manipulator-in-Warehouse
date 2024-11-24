from setuptools import setup
import os
from glob import glob

package_name = 'arm_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[
        'test_moving',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include other files like launch files if any
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Package for controlling the robotic arm',
    license='BSD-2-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_moving = arm_control.test_moving:main',
            'moving = arm_control.moving:main',
        ],
    },
)
