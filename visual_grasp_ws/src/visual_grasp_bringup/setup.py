from glob import glob
from setuptools import find_packages, setup


package_name = 'visual_grasp_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nav-Grasp Maintainers',
    maintainer_email='nav-grasp@users.noreply.github.com',
    description='Launch files for the visual grasp stack.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'model_joint_state_adapter = '
            'visual_grasp_bringup.model_joint_state_adapter:main',
        ],
    },
)

