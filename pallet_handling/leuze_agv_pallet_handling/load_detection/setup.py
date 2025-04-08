from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'load_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lepuser',
    maintainer_email='lepuser@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dbscan_object_detection = load_detection.dbscan_object_detection:main',
            'only_ortogonal = load_detection.only_ortogonal:main',
            'user_input = load_detection.user_input:main',
            'detector_panel = load_detection.detector_panel:main',
            'pallet_transformer = load_detection.pallet_transformer:main',
            'load_detection = load_detection.load_detection:main',
            'pallet_detection = load_detection.pallet_detection:main'
        ],
    },
)
