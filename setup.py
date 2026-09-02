import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'trash_modular'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rameel',
    maintainer_email='rameelamjad@gmail.com',
    description='Modular, independently-testable trash-sorting pipeline for the Hiwonder JetRover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Full pipeline
            'trash_sorter_pipeline = trash_modular.pipeline.trash_sorter_pipeline:main',

            # Independent module tests
            'test_camera = trash_modular.test_nodes.test_camera:main',
            'test_depth = trash_modular.test_nodes.test_depth:main',
            'test_imu = trash_modular.test_nodes.test_imu:main',
            'test_lidar = trash_modular.test_nodes.test_lidar:main',
            'test_detector = trash_modular.test_nodes.test_detector:main',
            'test_arm = trash_modular.test_nodes.test_arm:main',
            'test_gripper = trash_modular.test_nodes.test_gripper:main',
            'test_movement = trash_modular.test_nodes.test_movement:main',
            'test_alignment = trash_modular.test_nodes.test_alignment:main',
            'test_bin = trash_modular.test_nodes.test_bin:main',
            'test_pipeline = trash_modular.test_nodes.test_pipeline:main',

            # Calibration utilities
            'calibrate_home_bin = trash_modular.scripts.calibrate_home_bin:main',
            'calibrate_gripper = trash_modular.scripts.calibrate_gripper:main',
            'calibrate_grasp = trash_modular.scripts.calibrate_grasp:main',
        ],
    },
)
