from setuptools import find_packages, setup

package_name = 'camera_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera_launch.py']),
        ('share/' + package_name + '/resource', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joao Pedro',
    maintainer_email='joaorebolopedro@gmail.com',
    description='Camera Node for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = camera_node.camera_node:main'
        ],
    }
)
