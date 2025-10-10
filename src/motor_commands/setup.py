from setuptools import find_packages, setup

package_name = 'motor_commands'

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
    maintainer='pi2',
    maintainer_email='mernaiskander35@gmail.com',
    description='TODO: Package description',
    license='apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_commands = motor_commands.motor_commands:main',
            'transmit_command = motor_commands.transmit_command:main',
        ],
    },
)
