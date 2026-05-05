import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hri_workshop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Install world files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='polydorosa@gmail.com',
    description='HRI Workshop – Human Detection and Robot Approach with the Limo robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'human_detector = hri_workshop.human_detector:main',
            'proxemic_follower = hri_workshop.proxemic_follower:main',
        ],
    },
)
