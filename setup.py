from setuptools import find_packages, setup

package_name = 'aqua2_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kalvik',
    maintainer_email='itskalvik@gmail.com',
    description='A ROS2 package for controlling the Aqua2 robot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller = aqua2_control.controller:main'
        ],
    },
)
