from setuptools import find_packages, setup

package_name = 'camera_inspection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/damaged_objects', ['damaged_objects/damaged_objects_data.json']),
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
            'damaged_object_tracker = camera_inspection.damaged_object_tracker:main',
            'damaged_objects_rviz_publisher = camera_inspection.damaged_objects_rviz_publisher:main',
        ],
    },
)
