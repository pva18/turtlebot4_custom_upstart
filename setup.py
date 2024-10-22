from setuptools import setup
from setuptools import find_packages

from glob import glob
import os

package_name = 'turtlebot4_custom_upstart'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']), # install every folder except test
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soeroesg',
    maintainer_email='gabor.soros@gmail.com',
    description='Custom turtlebot4 upstart package',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vps_vio_node = turtlebot4_custom_upstart.vps_vio_node:main',
        ],
    },
)
