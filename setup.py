from setuptools import find_packages, setup

package_name = 'ros2websocket'

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
    maintainer='thomasluk',
    maintainer_email='thomasluk624@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2websocket_sender_node = ros2websocket.ros2websocket_sender_node:main',
            'ros2websocket_receiver_node = ros2websocket.ros2websocket_receiver_node:main',
        ],
    },
)
