from setuptools import setup, find_packages

package_name = 'shoulder_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='enxi@kth.se',
    description='Shoulder gait generator for CTSA quadruped',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'shoulder_gait = shoulder_controller.shoulder_gait_node:main',
        ],
    },
)
