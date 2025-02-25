import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'cw1_team_3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Renkai Liu',
    maintainer_email='renkai.liu.20@ucl.ac.uk',
    description='Coursework 1 team 3 solutions.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cw1_team_3 = cw1_team_3.obstacle_follower:main'
        ],
    },
)
