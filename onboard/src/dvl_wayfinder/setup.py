from pathlib import Path

from setuptools import find_packages, setup

package_name = 'dvl_wayfinder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [str(file) for file in Path('./config').glob('*.yaml')]),
        ('share/' + package_name + '/launch', [str(file) for file in Path('./launch').glob('*')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duke Robotics',
    maintainer_email='hello@duke-robotics.com',
    description='Obtain information from the DVL and publish it.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dvl_wayfinder = dvl_wayfinder.dvl_wayfinder:main',
        ],
    },
)
