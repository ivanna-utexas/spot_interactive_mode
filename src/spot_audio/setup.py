from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'spot_audio'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'sounddevice==0.5.1',
        'soxr==0.5.0',
        'faster-whisper==1.1.0',
        'ctranslate2==4.5.0',
        'numpy',
    ],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@example.com',
    description='Microphone capture and speech-to-text for Spot robot',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'audio_capture_node = spot_audio.audio_capture_node:main',
            'stt_node = spot_audio.stt_node:main',
            'discover_device = spot_audio.tools.discover_device:main',
            'record_wav = spot_audio.tools.record_wav:main',
        ],
    },
)
