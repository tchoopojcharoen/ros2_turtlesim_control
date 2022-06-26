from setuptools import setup
import os
from glob import glob

package_name = 'turtlesim_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch/*.launch.py')),
        (os.path.join('share',package_name,'config'),glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='teacher',
    maintainer_email='teacher@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlesim_controller=turtlesim_control.controller:main',
            'turtlesim_scheduler=turtlesim_control.scheduler:main',
            
            'turtle_follower=turtlesim_control.turtle_follower:main',
            'via_point_follower=turtlesim_control.via_point_follower:main',
            
        ],
    },
)