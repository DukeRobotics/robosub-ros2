from setuptools import find_packages, setup
import glob

package_name = 'cv'

# glob all config files in /config

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['cv', 'cv.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('./assets/*.png')),
        ('share/' + package_name + '/config', glob('./config/*.yaml')),
        ('share/' + package_name + '/config', glob('./launch/*.launch')),
        ('share/' + package_name + '/config', glob('./models/*.blob')),
        ('share/' + package_name + '/config', glob('./models/*.yaml')),

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
        ],
    },
)
