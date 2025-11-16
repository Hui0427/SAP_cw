from setuptools import setup

package_name = 'sfm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resources/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Structure from Motion (SfM) tutorial package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sfm_node = sfm.sfm_node:main',
        ],
    },
)