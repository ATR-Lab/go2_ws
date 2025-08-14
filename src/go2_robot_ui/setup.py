from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'go2_robot_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'numpy',
        'pyzbar',  # For QR code detection
    ],
    zip_safe=True,
    maintainer='atr-lab',
    maintainer_email='atr-lab@todo.todo',
    description='Go2 Robot PyQt UI for camera display and robot control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go2_robot_ui = go2_robot_ui.go2_ui_node:main',
        ],
    },
)
