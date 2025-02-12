from pathlib import Path

from setuptools import find_packages, setup

package_name = 'sensor_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (str(Path('share') / package_name / 'launch'), list(map(str, Path('./launch').glob('*.launch.py')))),
        (str(Path('share') / package_name / 'params'), list(map(str, Path('./params').glob('*.yaml')))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duke Robotics',
    maintainer_email='hello@duke-robotics.com',
    description=' Package that combines multiple sources of sensor data and uses an EKF for robot localization',
    license='MIT',
    entry_points={
        'console_scripts': [
        ],
    },
)
