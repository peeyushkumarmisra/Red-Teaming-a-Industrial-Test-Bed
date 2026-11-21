# =============================================================================
#  setup.py - Python package and its two ROS executables
# -----------------------------------------------------------------------------
#  plc_planner (the trusted PLC) and mitm (the attacker), installed for `ros2 run`.
# =============================================================================

from setuptools import find_packages, setup

package_name = 'ids_experiments'

setup(
    name=package_name,
    version='1.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Piyush Mishra',
    maintainer_email='im.pkm29@gmail.com',
    description='PLC-style planner, setpoint MitM implant and scenario schema for the IDS '
                'experiments.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plc_planner = ids_experiments.plc_planner:main',
            'mitm = ids_experiments.mitm_node:main',
        ],
    },
)
