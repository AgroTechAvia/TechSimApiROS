from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'computer_vision_functionality'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='senya',
    maintainer_email='senya@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "get_image_from_sim_node = computer_vision_functionality.get_image_from_sim_node:main",
            "recognition_of_aruco_marker_node = computer_vision_functionality.recognition_of_aruco_marker_node:main",
            "read_lidar_point_cloud_node = computer_vision_functionality.get_point_cloud_by_lidar_from_sim_node:main",
            "sim_driver_node = computer_vision_functionality.sim_driver:main",
            "teleop_node = computer_vision_functionality.teleop:main"
        ],
    },
)
