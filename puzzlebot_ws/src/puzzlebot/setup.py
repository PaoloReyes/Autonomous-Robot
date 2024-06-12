import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'puzzlebot'
submodules = 'puzzlebot/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rikunav',
    maintainer_email='rikunav@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_segmentation_node = puzzlebot.yolo_segmentation_node:main',
        ],
    },
)
