from setuptools import setup, find_packages

package_name = 'tb3_apf_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL@example.com',
    description='APF Planner for TurtleBot3 in ROS2',
    license='LICENSE_NAME',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # We define a console script so we can "ros2 run tb3_apf_planner apf_planner_node"
            'apf_planner_node = tb3_apf_planner.apf_planner_node:main',
        ],
    },
)
