from setuptools import find_packages, setup

package_name = 'robot_system_monitor'
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml', 'README.md']),
]

setup(
    name=package_name, version='0.0.0', packages=find_packages(exclude=['test']),
    data_files=data_files, install_requires=['setuptools'], zip_safe=False,
    maintainer='Project maintainer (TODO)', maintainer_email='maintainer@example.com',
    description='Diagnostyka robota i lokalny panel ROS 2.', license='TODO',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['monitor = robot_system_monitor.monitor:main']},
)
