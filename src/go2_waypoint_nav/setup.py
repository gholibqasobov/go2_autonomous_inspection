from setuptools import find_packages, setup

package_name = 'go2_waypoint_nav'

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
    maintainer='qasob',
    maintainer_email='qasobovgholib@gmail.com',
    description='Unitree Go2 ROS2 waypoint navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go2_waypoint_follower = go2_waypoint_nav.waypoint_follower:main',
            'go2_waypoint_saver = go2_waypoint_nav.waypoint_saver:main',
            'go2_waypoint_follower_odom = go2_waypoint_nav.waypoint_follower_odom:main',
            'go2_waypoint_follower_server = go2_waypoint_nav.waypoint_follower_server:main',
            'camera_servo_server = go2_waypoint_nav.camera_servo_server:main',
            'waypoint_client = go2_waypoint_nav.waypoint_follower_client:main',
        ],
    },
)
