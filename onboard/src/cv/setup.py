from setuptools import find_packages, setup

package_name = 'cv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['cv', 'cv.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Computer Vision pipeline, including depthai and hsv filtering.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bin_detector = cv.bin_detector:main',
            'blue_rectangle_detector = cv.blue_rectangle_detector:main',
            'buoy_detector_contour_matching = cv.buoy_detector_contour_matching:main',
            'depthai_camera_connect = cv.depthai_camera_connect:main',
            'depthai_mono_detection = cv.depthai_mono_detection:main',
            'depthai_publish_save_streams = cv.depthai_publish_save_streams:main',
            'depthai_spatial_detection = cv.depthai_spatial_detection:main',
            'path_marker_detector = cv.path_marker_detector:main',
            'pink_bins_detector = cv.pink_bins_detector:main',
            'usb_camera_connect = cv.usb_camera_connect:main',
            'usb_camera = cv.usb_camera:main',
        ],
    },
)
