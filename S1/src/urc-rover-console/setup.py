import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'urc_rover_console'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
	(
        	"share/ament_index/resource_index/packages",
        	["resource/" + package_name],
   	),
	(
        	"share/" + package_name,
        	["package.xml"],
    	),
    	(
        	os.path.join("share", package_name, "launch"),
        	glob("launch/*.launch.py"),
    	),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nipun Maisheri',
    maintainer_email='nmaisher@umd.edu',
    description='ROS 2 and PyQt operator dashboard for simulated URC autonomous navigation missions',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'rover_simulator = urc_rover_console.rover_simulator:main',
        	'battery_gui = urc_rover_console.battery_gui:main',
        ],
    },
)
