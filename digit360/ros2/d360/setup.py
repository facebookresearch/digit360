# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

from setuptools import find_packages, setup

package_name = 'd360'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=['d360', 'digit360']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/d360_launch.py', 'launch/d360_min_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chaitanyantr',
    maintainer_email='boddulurkrishna@meta.com',
    description='Digit 360 is a modular platform that unlocks new capabilities, and enables future research on the nature of touch.',
    license='CC BY-NC 4.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "serial_io = d360.d360_serial_io:main",
            "image_pub_opencv = d360.d360_image_pub:main",
            "audio_pub = d360.d360_audio_pub:main",
            "gas_pub = d360.d360_gas_pub:main",
        ],
    },
)