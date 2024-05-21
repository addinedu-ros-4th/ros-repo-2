from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'task_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='cheun0928@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'order_receiver = task_manager.order_receiver:main',
            'task_dispatcher = task_manager.task_dispatcher:main',
            'robot_controller = task_manager.robot_controller:main',
            'main_executable = task_manager.main:main',
            'robot_state_publisher = task_manager.test:main',
            'robot1 = task_manager.domain_test:main'
        ],
    },
)
