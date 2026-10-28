from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # *.xml matters: the persistent-memory BT was previously NOT
        # installed to share/ because the glob only matched *.yaml.
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml')) + glob(os.path.join('config', '*.xml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='todo@todo.com',
    description='Robot bringup launch files',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spatial_projector = robot_bringup.spatial_projector:main',
            'camera_memory = robot_bringup.camera_memory:main',
        ],
    },
)