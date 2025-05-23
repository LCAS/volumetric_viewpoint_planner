from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'viewpoint_generator'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #Adding launch files
        (os.path.join('share',package_name,'launch'),glob('launch/*.launch.*')),
        (os.path.join('share',package_name,'config'),glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'RViz'), glob('config/RViz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdurrahman Yilmaz',
    maintainer_email='ayilmaz@lincoln.ac.uk',
    description='Publish lattice points (viewpoints array)',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sampling_surface = viewpoint_generator.sampling_surface:main'
        ],
    },
)
