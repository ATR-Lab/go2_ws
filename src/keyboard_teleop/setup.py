from setuptools import setup

package_name = 'keyboard_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/keyboard_teleop_launch.py']),
        ('share/' + package_name + '/resource', ['resource/keyboard_teleop']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Keyboard teleoperation node using pynput for publishing Twist messages',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = keyboard_teleop.keyboard_teleop_node:main',
        ],
    },
)
