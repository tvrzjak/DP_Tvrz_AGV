from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'Leuze_omni_AGV_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'Leuze_omni_AGV_description'), glob('Leuze_omni_AGV_description/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lepuser',
    maintainer_email='Jakub.Tvrz@leuze.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	    'state_publisher = Leuze_omni_AGV_description.state_publisher:main',
	    'omni_kinematics = Leuze_omni_AGV_description.omni_kinematics:main'
        ],
    },
)
