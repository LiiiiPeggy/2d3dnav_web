from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'nav2_web'


setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        (
            'share/' + package_name + '/web',
            [path for path in glob('web/*') if os.path.isfile(path)],
        ),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='w',
    maintainer_email='w@example.com',
    description='Lightweight mobile landscape Web UI for ROS 2 Nav2.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'nav2_web_bridge = nav2_web.bridge:main',
        ],
    },
)
