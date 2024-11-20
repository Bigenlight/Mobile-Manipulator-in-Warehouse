from setuptools import setup
import os
from glob import glob

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
        # If you have launch files or other resources, include them here
        # ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Captures, undistorts, and publishes images from a camera.',
    license='Apache License 2.0',
    #tests_require=['pytest'],
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'cctv_node = cctv.cctv_node:main',
        ],
    },
)
