from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'dh5_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f"{package_name}.scripts"],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='DH5 ROS2 controller package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dh5_controller_node = dh5_controller.dh5_controller_node:main',

            'initialize_client = dh5_controller.scripts.initialize_client:main',

            'set_position_client = dh5_controller.scripts.set_position_client:main',
            'set_speed_client = dh5_controller.scripts.set_speed_client:main',
            'set_force_client = dh5_controller.scripts.set_force_client:main',

            'clear_cur_fault_client = dh5_controller.scripts.clear_cur_fault_client:main',
            'clear_history_faults_client = dh5_controller.scripts.clear_history_faults_client:main',
            'get_faults_client = dh5_controller.scripts.get_faults_client:main',

            'restart_system_client = dh5_controller.scripts.restart_system_client:main',

        ],
    },
)