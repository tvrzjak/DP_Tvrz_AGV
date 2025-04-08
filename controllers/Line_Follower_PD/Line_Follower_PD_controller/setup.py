from setuptools import setup
import os
from glob import glob

package_name = 'Line_Follower_PD_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jakub Tvrz', 
    maintainer_email='jakub.tvrz@leuze.com',
    description='ROS2 node for PD regulator from OGS',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follower_pd = Line_Follower_PD_controller.line_follower_pd:main',
        ],
    },
)