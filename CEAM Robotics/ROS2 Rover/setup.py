from setuptools import find_packages, setup

package_name = 'rover_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name + '/launch',
            ['launch/rover.launch.py', 'launch/gazebo.launch.py']
        ),
        (
            'share/' + package_name + '/worlds',
            ['worlds/empty.world']
        ),
        (
            'share/' + package_name + '/urdf',
            ['urdf/rover.urdf']
        ),
        (
            'share/' + package_name + '/models/rover',
            ['models/rover/model.sdf', 'models/rover/model.config']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Surya',
    description='ROS2 differential-drive rover simulation and control project',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'diff_drive_controller = rover_control.diff_drive_controller:main',
        ],
    },
)