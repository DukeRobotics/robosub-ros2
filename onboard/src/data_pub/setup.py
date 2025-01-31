from pathlib import Path

from setuptools import find_packages, setup

package_name = 'data_pub'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(include=['data_pub', 'data_pub.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', list(map(str, Path('./config').glob('*.yaml')))),
        ('share/' + package_name + '/launch', list(map(str,  Path('./launch').glob('*.xml')))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duke Robotics',
    maintainer_email='hello@duke-robotics.com',
    description='Package that reads and parses through sensor data.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dvl_raw = data_pub.dvl_raw:main',
            'dvl_odom = data_pub.dvl_to_odom:main',
            'peripheral = data_pub.peripheral:main',
        ],
    },
)
