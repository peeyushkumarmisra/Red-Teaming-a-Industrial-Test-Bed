from setuptools import find_packages, setup

package_name = 'ot_bridge'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ot_network.launch.py']),
        ('share/' + package_name + '/config', ['config/modbus_map.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Piyush Mishra',
    maintainer_email='im.pkm29@gmail.com',
    description='OT network bridge for the physics-informed manipulator IDS.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_gateway = ot_bridge.robot_gateway:main',
            'ids_replica = ot_bridge.ids_replica:main',
            'attacker_proxy = ot_bridge.attacker_proxy:main',
            'soft_plc = ot_bridge.soft_plc_node:main',
            'hmi = ot_bridge.hmi:main',
            'range_control = ot_bridge.range_control:main',
            'eng_ws = ot_bridge.eng_ws:main',
        ],
    },
)