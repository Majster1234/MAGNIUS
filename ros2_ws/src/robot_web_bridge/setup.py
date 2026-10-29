from os.path import relpath
from pathlib import Path

from setuptools import find_packages, setup

package_name = 'robot_web_bridge'
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml', 'README.md']),
]

web_dist = Path(__file__).resolve().parents[3] / 'web' / 'dist'
for directory in sorted({p.parent for p in web_dist.rglob('*') if p.is_file()}):
    data_files.append((str(Path('share') / package_name / 'web' / directory.relative_to(web_dist)),
                       [relpath(p, Path(__file__).resolve().parent)
                        for p in sorted(directory.iterdir()) if p.is_file()]))

setup(
    name=package_name, version='0.0.0', packages=find_packages(exclude=['test']),
    data_files=data_files, install_requires=['setuptools'], zip_safe=False,
    maintainer='Project maintainer (TODO)', maintainer_email='maintainer@example.com',
    description='Diagnostyka robota i lokalny panel ROS 2.', license='TODO',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['bridge = robot_web_bridge.bridge:main']},
)
