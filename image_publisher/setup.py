from setuptools import setup
import os
from glob import glob

package_name = 'image_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Include package.xml
        ('share/' + package_name, ['package.xml']),
        # Include all Python scripts
        (os.path.join('share', package_name, 'images'), glob('images/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 Image Publisher Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = image_publisher.image_publisher:main',
        ],
    },
)
