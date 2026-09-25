import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'forklift_diagnostics'

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
            'diagnostics_manager_node = forklift_diagnostics.diagnostics_manager_node:main',
            'hardware_monitor_node = forklift_diagnostics.hardware_monitor_node:main',
            'sensor_monitor_node = forklift_diagnostics.sensor_monitor_node:main',
            'system_monitor_node = forklift_diagnostics.system_monitor_node:main',
            'communication_monitor_node = forklift_diagnostics.communication_monitor_node:main',
            'maintenance_monitor_node = forklift_diagnostics.maintenance_monitor_node:main',
            'diagnostics_publisher_node = forklift_diagnostics.diagnostics_publisher_node:main',
        ],
    },
)
