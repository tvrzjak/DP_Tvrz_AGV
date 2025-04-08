from setuptools import setup
import os
from glob import glob

package_name = 'diff_robot_gazebo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf', 'mp_400'), glob('urdf/mp_400/*.urdf')),
        (os.path.join('share', package_name, 'urdf', 'mp_400', 'meshes'), glob('urdf/mp_400/meshes/*.dae')),
        (os.path.join('share', package_name, 'components', 'sensors'), glob('components/sensors/*.dae')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jakub Tvrz',
    maintainer_email='jtvrz@leuze.com',
    description='Differential robot simulation in Gazebo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['regulator = diff_robot_gazebo.regulator:main'],
    },
)
