from setuptools import find_packages, setup

package_name = 'path_servo_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Path to servo velocity controller for robotic arms.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 格式：'节点名 = 包名.文件名:main函数'
            'path_to_servo_controller = path_servo_control.path_to_servo_controller:main'
        ],
    },
)
