from setuptools import find_packages, setup

package_name = 'remote_server_nodes'

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
            'quadcopter_pilot = remote_server_nodes.quadcopter_pilot:main',
            'camera_follower = remote_server_nodes.follow_cam:main',
            'autonomous_pilot = remote_server_nodes.autonomous_pilot:main',
            'diagonal_pilot = remote_server_nodes.diagonal_pilot:main',
            'rl_pilot = remote_server_nodes.rl_pilot:main',
            'rl_test = remote_server_nodes.rl_test:main',
            'rl_plotter = remote_server_nodes.rl_plotter:main',
        ],
    },
)
