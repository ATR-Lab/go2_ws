from setuptools import find_packages, setup

package_name = 'go2_dance_orchestrator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/single_command_test.launch.py']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='atr-lab',
    maintainer_email='irvsteve@gmail.com',
    description='ROS2 package for orchestrating dance routines on Go2 robot with command completion tracking',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dance_orchestrator = go2_dance_orchestrator.presentation.dance_orchestrator_node:main',
        ],
    },
)
