from setuptools import setup

package_name = 'line_follower_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/control_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RDK Team',
    maintainer_email='maintainer@example.com',
    description='Differential drive control utilities for the RDK X5 line follower platform.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'differential_drive_controller = line_follower_control.differential_drive_controller:main',
        ],
    },
)
