from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'humanoid_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='you@example.com',
    description='Hardware driver for humanoid robot servo control via serial',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_driver = humanoid_hardware.servo_driver:main'
        ],
    },
)
