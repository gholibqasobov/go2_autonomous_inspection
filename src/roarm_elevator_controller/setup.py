from setuptools import find_packages, setup

package_name = 'roarm_elevator_controller'

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
    description='ROS2 RoArm Elevator Operation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'move_point_client = roarm_elevator_controller.move_point_client:main',
            'roarm_elevator_button_control = roarm_elevator_controller.roarm_elevator_button_control:main',
            'push_elevator_button_server = roarm_elevator_controller.push_elevator_button_server:main',
        ],
    },
)
