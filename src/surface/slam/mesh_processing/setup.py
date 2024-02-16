from setuptools import setup

package_name = 'mesh_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rov',
    maintainer_email='nrm98@case.edu',
    description='Stitches together pointcloud data into a single mesh',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mesh_processing_node = mesh_processing.mesh_processing:main',
        ],
    },
)
