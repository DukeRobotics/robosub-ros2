from pathlib import Path

from setuptools import find_packages, setup

package_name = 'offboard_comms'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [str(file) for file in Path('./config').glob('*.yaml')]),
        ('share/' + package_name + '/data', [str(file) for file in Path('./data').glob('*.csv')]),
        ('share/' + package_name + '/launch', [str(file) for file in Path('./launch').glob('*.xml')]),
        ('share/' + package_name + '/sketches',
         [str(file) for file in Path('./sketches').glob('*') if file.is_file()]),
        ('share/' + package_name + '/sketches/peripheral',
         [str(file) for file in Path('./sketches/peripheral').glob('*')]),
        ('share/' + package_name + '/sketches/thruster',
         [str(file) for file in Path('./sketches/thruster').glob('*')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duke Robotics',
    maintainer_email='hello@duke-robotics.com',
    description='Obtain sensor information from Arduinos and send thruster commands.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arduino = offboard_comms.arduino:main',
            'dvl_raw = offboard_comms.dvl_raw:main',
            'dvl_odom = offboard_comms.dvl_to_odom:main',
            'peripheral = offboard_comms.peripheral:main',
            'test_thrusters = offboard_comms.test_thrusters:main',
            'thrusters = offboard_comms.thrusters:main',
        ],
    },
)
