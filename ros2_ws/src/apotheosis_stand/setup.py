from setuptools import find_packages, setup

package_name = 'apotheosis_stand'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/stand.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/stand_arm.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics Labs',
    maintainer_email='student@example.com',
    description='Учебный ROS 2-стенд манипулятора: tf2, гомография, IK',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_flight = apotheosis_stand.camera_flight:main',
            'marker_tracker = apotheosis_stand.marker_tracker:main',
            'ik_server = apotheosis_stand.ik_server:main',
            'bridge_server = apotheosis_stand.bridge_server:main',
            'arm_viz = apotheosis_stand.arm_viz:main',
            'urdf_pub = apotheosis_stand.urdf_pub:main',
        ],
    },
)
