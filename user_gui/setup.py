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
        # Include any additional resources
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'PyQt5',
        'opencv-python',
        'numpy',
        'python-dotenv',
        'cv_bridge',  # Added cv_bridge
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Combined GUI Application',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'user_gui = user_gui.user_gui:main',
            'yolo_publisher = user_gui.yolo_publisher:main',
            'image_subscriber = user_gui.image_subscriber:main',
            'yolo_image = user_gui.yolo_image:main'
        ],
    },
)
