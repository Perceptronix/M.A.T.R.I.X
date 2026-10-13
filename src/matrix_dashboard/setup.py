import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'matrix_dashboard'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matrix',
    maintainer_email='noreply@example.com',
    description='M.A.T.R.I.X live engineering dashboard (real ROS 2 data only).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_node = matrix_dashboard.dashboard_node:main',
            'elevation_cloud_bridge = matrix_dashboard.elevation_cloud_bridge:main',
        ],
    },
)
