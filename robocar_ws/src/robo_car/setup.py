import os
from glob import glob
from setuptools import setup

package_name = 'robo_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Stacy',
    maintainer_email='catundertheleaf@gmail.com',
    description='A package to control a car.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robocar = robo_car.robocar:main'
        ],
    },
)
