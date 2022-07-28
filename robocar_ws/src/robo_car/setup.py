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
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
        (os.path.join('share', package_name, 'model'), glob('world/prius_hybrid/*.urdf'))
        # (os.path.join('share', package_name), glob('world/**', recursive=True))
        # ('target_directory_1', glob('source_dir/*')), # files in source_dir only - not recursive
        # ('target_directory_2', glob('nested_source_dir/**/*', recursive=True)), # includes sub-folders - recursive
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
            'robocar = robo_car.robocar:main',
            'mock_camera = robo_car.mock_camera:main'
        ],
    },
)
