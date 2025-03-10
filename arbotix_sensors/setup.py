#!/usr/bin/env python3

import os
from glob import glob
from setuptools import setup

package_name = 'arbotix_sensors'

setup(
    name=package_name,
    version='0.11.0',
    # Packages to export
    packages=[package_name],
    # Files we want to install, specifically launch files
    #data_files=[
        # Install marker file in the package index
    #    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
    #    (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
    #    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    #],
    package_dir={"": 'src/'},
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    author='Michael Ferguson',
    author_email='mike@vanadiumlabs.com',
    maintainer='Michael Ferguson',
    maintainer_email='mike@vanadiumlabs.com',
    keywords=['arbotix'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: BSD',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Extends the arbotix_node package with a number of more sophisticated ROS wrappers for common devices.',
    license='BSD',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
            'ir_ranger = arbotix_sensors.ir_ranger:main',
            'max_sonar = arbotix_sensors.max_sonar:main'
        ],
    },
)
