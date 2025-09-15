from setuptools import find_packages, setup

package_name = 'sui_qr_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sui_qr_processor.launch.py']),
        ('share/' + package_name + '/config', ['config/sui_qr_processor.yaml']),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'pyzbar',
        'sui-py',
        'asyncio',
    ],
    zip_safe=True,
    maintainer='kpatch',
    maintainer_email='irvsteve@gmail.com',
    description='ROS2 package for processing QR codes from camera feed and executing Sui blockchain transactions',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sui_qr_processor_node = sui_qr_processor.sui_qr_processor_node:main',
        ],
    },
)
