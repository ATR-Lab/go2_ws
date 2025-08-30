from setuptools import find_packages, setup

package_name = 'scan_restamper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo.todo@todo.todo',
    description='ROS2 package for restamping laser scan messages',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_restamper_node = scan_restamper.scan_restamper_node:main',
            'restamp_nav_actions = scan_restamper.restamp_nav_actions:main',
        ],
    },
)
