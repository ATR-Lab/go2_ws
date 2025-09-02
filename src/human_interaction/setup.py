from setuptools import setup

package_name = 'human_interaction'

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
    maintainer='atr-lab',
    maintainer_email='atr-lab@example.com',
    description='Human detection and interaction management for robot dog petting zoo system',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'human_detection_node = human_interaction.human_detection_node:main',
            'interaction_manager_node = human_interaction.interaction_manager_node:main',
            'gesture_recognition_node = human_interaction.gesture_recognition_node:main',
        ],
    },
)
