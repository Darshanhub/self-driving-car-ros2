from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonomous_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'numpy', 'psutil', 'pynvml', 'pyyaml'],
    zip_safe=True,
    maintainer='Darshan AI Lab',
    maintainer_email='autonomy@example.com',
    description='ROS 2 perception stack wrapping pretrained BEV/occupancy networks.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bev_fusion_node = autonomous_perception.bev_fusion_node:main',
            'profile_logger = autonomous_perception.scripts.profile_runner:main',
            'nuscenes_to_mcap = autonomous_perception.scripts.nuscenes_to_mcap:main',
        ],
    },
)
