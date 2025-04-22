from setuptools import find_packages, setup

package_name = 'elevator_controller'

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
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'elevator_controller_client = elevator_controller.elevator_controller_client:main',
            'apriltag_docking_client = elevator_controller.apriltag_docking_client:main',
            'elevator_button_push_client = elevator_controller.elevator_button_push_client:main',
        ],
    },
)
