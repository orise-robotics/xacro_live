# from glob import glob
import os

from setuptools import setup

package_name = 'xacro_live'

setup(
    name=package_name,
    version='0.0.1',
    # Packages to export
    packages=[package_name],
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # # Include all launch files.
        # (os.path.join('share', package_name, 'launch'), glob('*.launch.py'))
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    author='Mateus Amarante Araujo',
    author_email='mateus.amarujo@gmail.com',
    maintainer='Mateus Amarante, Ivan Tarifa, Caio Amaral',
    maintainer_email='mateus.amarujo@gmail.com, ivan1993br@gmail.com, caioaamaral@gmail.com',
    keywords=['ros2', 'xacro', 'robot_description'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Update the robot_description dinamically from updates on a target xacro file.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'xacro_live = xacro_live.xacro_live_node:main'
        ],
    },
)
