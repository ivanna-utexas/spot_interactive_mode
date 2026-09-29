import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'spot_interactive_mode'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ivanna Aleman-Coronado',
    maintainer_email='ivannaac@utexas.edu',
    description='Switchable STAY / FOLLOW / DANCE interactive mode for Spot.',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'interactive_mode_node = spot_interactive_mode.interactive_mode_node:main',
            'mode_keyboard = spot_interactive_mode.mode_keyboard:main',
        ],
    },
)
