from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'py_robotsim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cassia',
    maintainer_email='cassia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator = py_robotsim.simulator:main',
            'translator = py_robotsim.translator:main',
            'map = py_robotsim.map:main',
            'nav_control = py_robotsim.nav_control:main'
        ],
    },
)
