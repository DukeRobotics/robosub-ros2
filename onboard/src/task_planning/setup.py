import os
from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = 'task_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (Path('share') / package_name / 'launch', Path.glob('./launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duke Robotics',
    maintainer_email='hello@duke-robotics.com',
    description='Task planning package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_runner = task_planning.task_runner:main',
        ],
    },
)
