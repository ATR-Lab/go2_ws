from setuptools import find_packages, setup

package_name = 'navigation_manager'

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
    maintainer='atr-lab',
    maintainer_email='irvsteve@gmail.com',
    description='Navigation manager for coordinating Nav2 with human interactions in robot dog petting zoo',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_coordinator = navigation_manager.navigation_coordinator:main',
        ],
    },
)
