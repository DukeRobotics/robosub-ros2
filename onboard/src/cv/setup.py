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
    description='Computer Vision pipeline, including depthai and hsv filtering.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bin_detector = scripts.bin_detector:main',
            'blue_rectangle_detector = scripts.blue_rectangle_detector:main',
            'buoy_detector_contour_matching = scripts.buoy_detector_contour_matching:main',
            'depthai_camera_connect = scripts.depthai_camera_connect:main',
            'depthai_mono_detection = scripts.depthai_mono_detection:main',
            'depthai_publish_save_streams = scripts.depthai_publish_save_streams:main',
            'depthai_spatial_detection = scripts.depthai_spatial_detection:main',
            'path_marker_detector = scripts.path_marker_detector:main',
            'pink_bins_detector = scripts.pink_bins_detector:main',
            'usb_camera_connect = scripts.usb_camera_connect:main',
            'usb_camera = scripts.usb_camera:main',
        ],
    },
)
