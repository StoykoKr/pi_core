from setuptools import find_packages, setup

package_name = 'mqtt_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/bridge_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Petko Stoev',
    maintainer_email='petko_ivanov_stoev@abv.bg',
    description='Mobile app MQTT <-> ROS2 bridge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mqtt_bridge = mqtt_bridge.bridge:main',
        ],
    },
)
