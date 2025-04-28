from setuptools import find_packages, setup

package_name = 'hardware_interfaces'

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
    maintainer='rosdev',
    maintainer_email='diloncarnie@gmail.com',
    description='Hardware Interface Nodes for Systems Project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ultrasonic = hardware_interfaces.ultrasonic_sensor:main',
            'speaker = hardware_interfaces.voice_sensor:main',
            'manipulator = hardware_interfaces.manipulator_actuator:main',
        ],
    },
)
