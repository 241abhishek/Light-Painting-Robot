from setuptools import find_packages, setup

package_name = 'turtlebot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhi2001',
    maintainer_email='241abhishek@gmail.com',
    description='Control nodes for the turtlebot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "control = turtlebot_control.control:main"
        ],
    },
)
