from setuptools import find_packages, setup


package_name = 'robot_pose_estimation'

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
    maintainer='rodri',
    maintainer_email='iamrodrigopassos@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_estimation = robot_pose_estimation.pose_estimation_node:main',
            'odom_cam = robot_pose_estimation.odom_cam_node:main',
            'camera_config = robot_pose_estimation.camera_config:main',
        ],
    },
)
