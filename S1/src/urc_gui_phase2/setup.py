import os

from setuptools import find_packages, setup

package_name = 'urc_gui_phase2'


def tile_data_files(src_root='tiles'):
    """Offline map tiles (incl. metadata.json) -> share/<pkg>/tiles/..., keeping the z/x/y layout."""
    entries = []
    for dirpath, _dirs, files in sorted(os.walk(src_root)):
        files = sorted(f for f in files if not f.endswith('.part'))
        if files:
            entries.append((os.path.join('share', package_name, dirpath),
                            [os.path.join(dirpath, f) for f in files]))
    return entries


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + tile_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soham',
    maintainer_email='aggarwalsoham2008@gmail.com',
    description='Phase 2 (S1 Navigation GUI) for the URC robotics club challenge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_sim_node = urc_gui_phase2.rover_sim_node:main',
            'operator_gui_node = urc_gui_phase2.operator_gui_node:main',
        ],
    },
)
