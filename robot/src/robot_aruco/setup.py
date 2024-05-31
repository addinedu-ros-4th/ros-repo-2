from setuptools import find_packages, setup
import glob
import os

package_name = 'robot_aruco'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yongtak_ras',
    maintainer_email='dknjy3313@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detection = robot_aruco.aruco_detection:main',
            'robot_cam = robot_aruco.robot_cam:main',
            'step_control = robot_aruco.step_control:main',
<<<<<<< HEAD
            'button_lcd_control = robot_aruco.button_lcd_control:main',
            
=======
            'button_lcd_control = robot_aruco.button_lcd_control:main'
>>>>>>> 921ba49432eada22fb415fcc6c98d24afe8669cf
        ],
    },
)

