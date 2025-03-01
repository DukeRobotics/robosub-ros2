from pathlib import Path

from setuptools import find_packages, setup

package_name = 'sonar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [str(p) for p in Path('./launch').glob('*')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duke Robotics',
    maintainer_email='hello@duke-robotics.com',
    description='Run the Sonar on the robot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sonar = sonar.sonar:main',
            'sonar_test_client = sonar.sonar_test_client:main',
        ],
    },
)
