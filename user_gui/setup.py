from setuptools import setup
import os
from glob import glob

package_name = 'user_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Include package.xml
        ('share/' + package_name, ['package.xml']),
        # Include all images
        (os.path.join('share', package_name, 'images'), glob(os.path.join('user_gui', 'images', '*'))),
    ],
    install_requires=['setuptools', 'rclpy', 'PyQt5'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Restaurant Service Robot GUI using ROS2 Services',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'user_gui = user_gui.user_gui:main',
        ],
    },
)
