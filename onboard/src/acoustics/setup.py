from pathlib import Path

from setuptools import find_packages, setup

package_name = 'acoustics'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [str(file) for file in Path('./config').glob('*.yaml')]),
        ('share/' + package_name + '/launch', [str(p) for p in Path('./launch').glob('*')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duke Robotics',
    maintainer_email='hello@duke-robotics.com',
    description='Acoustics Service',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'acoustics = acoustics.acoustics:main',
        ],
    },
)
