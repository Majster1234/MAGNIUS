from setuptools import find_packages, setup

setup(name='robot_configuration', version='0.0.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/robot_configuration']),
                  ('share/robot_configuration', ['package.xml', 'README.md'])],
      install_requires=['setuptools'], zip_safe=False,
      maintainer='Project maintainer (TODO)', maintainer_email='maintainer@example.com',
      description='Konfiguracja robota i snapshoty JSON.', license='TODO', tests_require=['pytest'],
      entry_points={'console_scripts': ['configuration = robot_configuration.node:main']})
