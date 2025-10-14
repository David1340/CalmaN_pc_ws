from setuptools import find_packages, setup

package_name = 'exp_plots'

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
    maintainer='rodrigopassos',
    maintainer_email='iamrodrigopassos@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'trajectory = exp_plots.trajectory_plot:main',
            'velocity = exp_plots.velocity_plot:main',
            'pose = exp_plots.pose_plot:main',
            'encoder = exp_plots.encoder_plot:main',
            'freq_odom = exp_plots.freq_odom_plot:main',
        ],
    },
)
