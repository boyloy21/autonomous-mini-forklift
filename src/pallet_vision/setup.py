from setuptools import find_packages, setup
import glob

package_name = 'pallet_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name + '/models', glob('models/*.pt')),
        (
            "share/" + package_name + "/models",
            glob.glob(package_name + "/model_checkpoints/*.pt"),
        ),
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
            'pallet_detection = pallet_vision.pallet_detection:main',
            'pallet_detection2 = pallet_vision.pallet_detectionV2:main',
            # 'publish_image = pallet_vision.publishImage:main',
            # 'inference_node = pallet_vision.inference:main',
        ],
    },
)
