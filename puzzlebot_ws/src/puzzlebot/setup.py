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
            'camera_segmentation_node = puzzlebot.camera_segmentation_node:main'
        ],
    },
)
