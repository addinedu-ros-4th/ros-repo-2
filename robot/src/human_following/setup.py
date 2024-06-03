from setuptools import find_packages, setup
import glob
import os

package_name = 'human_following'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kkyu_rasp',
    maintainer_email='kkyu_rasp@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_cam = human_following.robot_cam:main',
            'following = human_following.following:main',
        ],
    },
)
