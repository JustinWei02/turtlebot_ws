from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'turtlebot3_pid_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=["test"]),  
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'nav_msgs',
        'sensor_msgs',
        'std_msgs',
        'transforms3d'
    ],  
    zip_safe=True,
    maintainer='justin-wei',
    maintainer_email='justinw02@yahoo.com',
    description='TurtleBot3 PID Controller in ROS 2 Jazzy',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_controller = pidcontroller:main'  # Corrected reference
        ],
    },
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('lib', package_name), glob('scripts/*.py')),  # Install scripts properly
    ],
)
