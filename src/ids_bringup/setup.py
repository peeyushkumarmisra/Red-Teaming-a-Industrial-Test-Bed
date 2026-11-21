# =============================================================================
#  setup.py - Python package, launch files, configs and two executables
# -----------------------------------------------------------------------------
#  run_supervisor (ends and summarises one run) and run_campaign (runs many).
# =============================================================================

from glob import glob

from setuptools import find_packages, setup

package_name = 'ids_bringup'

setup(
    name=package_name,
    version='1.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Piyush Mishra',
    maintainer_email='im.pkm29@gmail.com',
    description='Launch files, bridges, payload spawn, experiment runs, run supervision and '
                'the experiment campaign for the simulated iiwa cell.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_supervisor = ids_bringup.supervisor_node:main',
            'run_campaign = ids_bringup.campaign_runner:main',
        ],
    },
)
