from setuptools import setup

package_name = 'camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'numpy',
        'cv_bridge',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Package that streams video from webcam',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'robot_cam = camera.robot_cam:main',
        ],
    },
)
