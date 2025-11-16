from setuptools import setup
import os
from glob import glob

package_name = 'moveit_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resources/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), 
            glob('config/*')),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='MoveIt tutorial package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'moveit_demo = moveit_demo.moveit_demo:main',
            'grasp_from_ply   = moveit_demo.grasp_from_ply:main',
        ],
    },
)


