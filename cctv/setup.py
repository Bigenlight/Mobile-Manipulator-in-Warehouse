from setuptools import setup
from glob import glob
import os

package_name = 'cctv'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # Remove py_modules since cctv_node.py is inside the cctv package
    # py_modules=['cctv_node'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files if you have any
        # ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'numpy',
        # 'ultralytics',  # Uncomment if you use YOLO
        # 'shapely',      # Uncomment if used elsewhere
        # 'flask',        # Remove if not used
    ],

    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Captures, undistorts, and publishes images from a camera.',
    license='Apache License 2.0',
    # tests_require=['pytest'],  # Uncomment if you have tests
    entry_points={
        'console_scripts': [
            'cctv_node = cctv.cctv_node:main',
        ],
    },
)
