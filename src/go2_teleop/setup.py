from setuptools import setup

package_name = 'go2_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/xbox_teleop.launch.py']),
        ('share/' + package_name + '/resource', ['resource/go2_teleop']),
    ],
    install_requires=['setuptools', 'pygame'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Go2 robot teleoperation package supporting Xbox controller and keyboard input',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_xbox_controller = go2_teleop.xbox_teleop_node:main',
            'teleop_keyboard = go2_teleop.keyboard_teleop_node:main',
        ],
    },
)
