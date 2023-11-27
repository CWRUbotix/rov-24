import os
import sys
from glob import glob

from setuptools import setup

major_num = sys.version_info[0]
minor_num = sys.version_info[1]

package_name = 'manipulators'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools', 'flake8==4.0.1', 'mypy >= 1.7', 'smbus2'],
    zip_safe=True,
    maintainer='Georgia Martinez, Michael Carlstrom',
    maintainer_email='gcm49@case.edu, rmc170@case.edu',
    description='Code for manipulators',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
