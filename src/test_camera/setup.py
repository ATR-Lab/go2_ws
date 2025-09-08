from setuptools import setup
import os
from glob import glob

package_name = 'test_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atr-lab',
    maintainer_email='atr-lab@example.com',
    description='Test camera and visualization utilities for robot dog petting zoo system',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_camera_node = test_camera.test_camera_node:main',
            'test_camera_threaded_node = test_camera.test_camera_threaded_node:main',
            'simple_camera_display = test_camera.simple_camera_display:main',
        ],
    },
)
