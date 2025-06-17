from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'steve_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='r1',
    maintainer_email='r1@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'logged_waypoint_follower = steve_interface.logged_waypoint_follower:main',
            'interactive_waypoint_follower = steve_interface.interactive_waypoint_follower:main',
            'gps_wp_logger = steve_interface.gps_wp_logger:main',
            'main_controller = steve_interface.main_controller:main',
            
        ],
    },
)
