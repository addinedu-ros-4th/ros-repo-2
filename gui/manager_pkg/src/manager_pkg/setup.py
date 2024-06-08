from setuptools import find_packages, setup
import glob
import os

package_name = 'manager_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'image'), glob.glob('image/*')),
        (os.path.join('share', package_name, 'data'), glob.glob('data/*')),
        (os.path.join('share', package_name, 'ui'), glob.glob('ui/*')),
        (os.path.join('share', package_name, 'map'), glob.glob('map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='dydxkr0410@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manager_main = manager_pkg.manager_main:main',
            'manager_test = manager_pkg.manager_test:main'   
        ],
    },
)
