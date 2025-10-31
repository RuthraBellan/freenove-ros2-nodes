from setuptools import find_packages, setup

package_name = 'freenove_car'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'camera_node = freenove_car.camera_node:main',
        'motor_control_node = freenove_car.motor_control_node:main',
        'lane_follower_basic = freenove_car.lane_follower_node:main',
        'lane_follower_advanced = freenove_car.lane_follower_advanced_node:main',
    ],
},
)
