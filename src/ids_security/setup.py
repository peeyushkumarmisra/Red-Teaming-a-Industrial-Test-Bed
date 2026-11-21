from glob import glob

from setuptools import find_packages, setup

package_name = 'ids_security'

setup(
    name=package_name,
    version='1.6.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Piyush Mishra',
    maintainer_email='im.pkm29@gmail.com',
    description='SROS2 policy, keystore tools and overhead probe for the cell.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'make_keystore = ids_security.make_keystore:main',
            'check_keystore = ids_security.check_keystore:main',
            'probe = ids_security.probe:main',
            'overhead = ids_security.overhead:main',
        ],
    },
)