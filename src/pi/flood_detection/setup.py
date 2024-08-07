from setuptools import setup
from glob import glob
import os

package_name = 'flood_detection'

setup(
    name=package_name,
    version='1.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Carlstrom',
    maintainer_email='rmc@carlstrom.com',
    description='Flood detection',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flood_detector = flood_detection.flood_detector:main',
            'dry_run = flood_detection.gpio_reader_dry_run:main'
        ],
    },
)
