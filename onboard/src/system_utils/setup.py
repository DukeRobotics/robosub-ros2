from pathlib import Path

from setuptools import find_packages, setup

package_name = 'system_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [str(file) for file in Path('./launch').glob('*.xml')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duke Robotics',
    maintainer_email='hello@duke-robotics.com',
    description='Package that provides various system utilities.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'record_bag = system_utils.record_bag:main',
            'system_info_publisher = system_utils.system_info_publisher:main',
            'topic_transforms = system_utils.topic_transforms:main',
        ],
    },
)
