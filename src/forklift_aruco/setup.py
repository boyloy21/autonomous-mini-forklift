from setuptools import find_packages, setup
import os
import os
from glob import glob

package_name = 'forklift_aruco'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yunboyloy',
    maintainer_email='yunboyloy@aifarm.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_node = forklift_aruco.aruco_node:main',
            "generate_marker_image = forklift_aruco.generate_marker_image:main",
            "generate_marker_cube = forklift_aruco.generate_marker_cube:main",
            "calibration_camera = forklift_aruco.calibration_camera:main",
            "detect_aruco_image = forklift_aruco.detect_aruco_image:main",
            "detect_aruco_video = forklift_aruco.detect_aruco_video:main",
            "pose_estimate = forklift_aruco.pose_estimate:main",
        ],
    },
)
