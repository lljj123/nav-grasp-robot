from glob import glob
from setuptools import find_packages, setup


package_name = 'visual_grasp_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Nav-Grasp Maintainers',
    maintainer_email='nav-grasp@users.noreply.github.com',
    description='AprilTag eye-in-hand calibration for PiPER and D435.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'handeye_calibrator = visual_grasp_calibration.handeye_node:main',
            'handeye_collect = visual_grasp_calibration.handeye_collect:main',
            'handeye_tf_publisher = visual_grasp_calibration.handeye_tf_publisher:main',
            'piper_pose_tf_bridge = visual_grasp_calibration.piper_pose_tf_bridge:main',
        ],
    },
)

