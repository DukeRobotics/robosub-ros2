from pathlib import Path

from setuptools import find_packages, setup

package_name = 'static_transforms'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [str(file) for file in Path('./config').glob('*.yaml')]),
        (str(Path('share') / package_name / 'launch'), list(map(str, Path('launch').glob('*.launch.py')))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duke Robotics',
    maintainer_email='hello@duke-robotics.com',
    description='This package publishes the static transforms that define key parts of the robot.',
    license='MIT',
)
