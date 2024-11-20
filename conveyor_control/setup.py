# setup.py

from setuptools import setup

package_name = 'conveyor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',  # 작성자 이름
    maintainer_email='your_email@example.com',  # 작성자 이메일
    description='ROS2 package to control conveyor via Arduino using serial communication',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'order_belt_control = conveyor_control.order_belt_control:main',
        ],
    },
)
