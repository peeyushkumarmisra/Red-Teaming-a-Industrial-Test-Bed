# =============================================================================
#  setup.py - Python package with no executables
# -----------------------------------------------------------------------------
#  A library only: ids_experiments' MitM node imports it.
# =============================================================================

from setuptools import find_packages, setup

package_name = 'ids_attacks'

setup(
    name=package_name,
    version='1.4.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Piyush Mishra',
    maintainer_email='im.pkm29@gmail.com',
    description='Transport-agnostic attack library for the IDS experiments.',
    license='Apache-2.0',
    tests_require=['pytest'],
)
