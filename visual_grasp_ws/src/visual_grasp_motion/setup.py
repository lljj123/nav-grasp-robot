from glob import glob
from setuptools import find_packages, setup


package_name = 'visual_grasp_motion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Nav-Grasp Maintainers',
    maintainer_email='nav-grasp@users.noreply.github.com',
    description='Guarded grasp planning and two-stage PiPER execution.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'grasp_candidate_planner = visual_grasp_motion.grasp_candidate_planner:main',
            'piper_pick_executor = visual_grasp_motion.piper_pick_executor:main',
        ],
    },
)

