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
        (os.path.join('share', package_name), ['package.xml']),
        # Include images in share/image_publisher/images
        (os.path.join('share', package_name, 'images'), glob('images/*')),
    ],
    install_requires=['setuptools', 'ament_index_python'],  # Added 'ament_index_python'
    zip_safe=True,
    maintainer='theo',
    maintainer_email='tpingouin@gmail.com',
    description='ROS2 Image Publisher Package',
    license='Apache License 2.0',  # Ensure this matches package.xml
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = image_publisher.image_publisher:main',
        ],
    },
)
