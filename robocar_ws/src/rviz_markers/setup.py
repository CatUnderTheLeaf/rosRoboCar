from setuptools import setup

package_name = 'rviz_markers'

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
    maintainer='Stacy',
    maintainer_email='catundertheleaf@gmail.com',
    description='Package for rviz markers',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_publisher = rviz_markers.marker_publisher:main'
        ],
    },
)
