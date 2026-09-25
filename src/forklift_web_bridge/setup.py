import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'forklift_web_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
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
            'bridge_manager_node = forklift_web_bridge.bridge_manager_node:main',
            'robot_state_node = forklift_web_bridge.robot_state_node:main',
            'navigation_bridge_node = forklift_web_bridge.navigation_bridge_node:main',
            'mission_bridge_node = forklift_web_bridge.mission_bridge_node:main',
            'diagnostics_bridge_node = forklift_web_bridge.diagnostics_bridge_node:main',
            'mqtt_bridge_node = forklift_web_bridge.mqtt_bridge_node:main',
            'fastapi_bridge_node = forklift_web_bridge.fastapi_bridge_node:main',
        ],
    },
)
