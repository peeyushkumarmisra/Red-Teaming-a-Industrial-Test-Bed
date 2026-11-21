# =============================================================================
#  setup.py - Python package and its five commands
# -----------------------------------------------------------------------------
#  ros2 run ids_evaluation export_run | evaluate | compare_baselines | replay_run |
#  split_legacy (see cli.py).
# =============================================================================

from setuptools import find_packages, setup

package_name = 'ids_evaluation'

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
    description='Offline evaluation of the IDS.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'export_run = ids_evaluation.cli:export_main',
            'evaluate = ids_evaluation.cli:evaluate_main',
            'compare_baselines = ids_evaluation.cli:compare_main',
            'replay_run = ids_evaluation.cli:replay_main',
            'split_legacy = ids_evaluation.cli:legacy_main',
        ],
    },
)
