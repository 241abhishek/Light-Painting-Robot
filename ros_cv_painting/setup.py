from setuptools import find_packages, setup

package_name = 'ros_cv_painting'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/light_painting.launch.xml']),
         ('share/' + package_name + '/calibration_values',
         ['calibration_values/calibration_values.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhi2001',
    maintainer_email='241abhishek@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cv_light_painting = ros_cv_painting.cv_light_painting:main',
        ],
    },
)
