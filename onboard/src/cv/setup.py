from pathlib import Path

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
        (str(Path('share') / package_name / 'assets'), list(map(str, Path('./assets').glob('*.png')))),
        (str(Path('share') / package_name / 'config'), list(map(str, Path('./config').glob('*.yaml')))),
        (str(Path('share') / package_name / 'launch'), list(map(str, Path('./launch').glob('*.xml')))),
        (str(Path('share') / package_name / 'models'), list(map(str, Path('./models').glob('*.blob')))),
        (str(Path('share') / package_name / 'models'), list(map(str, Path('./models').glob('*.yaml')))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duke Robotics',
    maintainer_email='hello@duke-robotics.com',
    description='Computer Vision pipeline, including DepthAI and HSV filtering.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bin_detector = cv.bin_detector:main',
            'lane_marker_detector = cv.lane_marker_detector:main',
            'buoy_detector_contour_matching = cv.buoy_detector_contour_matching:main',
            'depthai_camera_connect = cv.depthai_camera_connect:main',
            'depthai_publish_save_streams = cv.depthai_publish_save_streams:main',
            'depthai_spatial_detection = cv.depthai_spatial_detection:main',
            'depthai_usb_detection = cv.depthai_usb_detection:main',
            'path_marker_detector = cv.path_marker_detector:main',
            'pink_bins_detector = cv.pink_bins_detector:main',
            'usb_camera_connect_all = cv.usb_camera_connect_all:main',
            'usb_camera = cv.usb_camera:main',
        ],
    },
)
