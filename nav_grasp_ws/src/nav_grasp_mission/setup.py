from glob import glob
from setuptools import find_packages, setup


package_name = 'nav_grasp_mission'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nav-Grasp Maintainers',
    maintainer_email='nav-grasp@users.noreply.github.com',
    description=(
        'YASMIN mission orchestration for Nav2 navigation, visual grasp, '
        'and return.'),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = nav_grasp_mission.mission_node:main',
        ],
    },
)
