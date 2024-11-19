from setuptools import setup
import os
from glob import glob

package_name = 'order_belt_listener'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 필요한 경우 launch 파일 추가
        # (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@example.com',
    description='A ROS2 node to listen to /order and /belt topics and parse messages.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'order_belt_listener = order_belt_listener.order_belt_listener:main',
        ],
    },
)
