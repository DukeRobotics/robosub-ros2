from setuptools import find_packages, setup
import glob

package_name = 'data_pub'

# glob all config files in /config
config_files = glob.glob('config/*.yaml')

if len(config_files) == 0:
    raise FileNotFoundError('No config files found in /config')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['data_pub', 'data_pub.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', config_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dvl_raw = data_pub.dvl_raw:main',
        ],
    },
)
