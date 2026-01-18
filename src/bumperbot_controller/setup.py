import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'bumperbot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kingade',
    maintainer_email='kogodemilade@gmail.com',
    description='BumperBot controller package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_controller = bumperbot_controller.simple_controller:main',
            'noisy_controller = bumperbot_controller.noisy_controller:main',
        ],
    },
)