from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'minibot_arucomarker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yongtak_ras',
    maintainer_email='yongtak_ras@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_aruco = minibot_arucomarker.detect_aruco:main',
            'img_publisher =minibot_arucomarker.img_publisher:main',
            'detect_aruco2 = minibot_arucomarker.detect_aruco2:main',
            'detect_aruco3 = minibot_arucomarker.detect_aruco3:main'
        ],
    },
)
