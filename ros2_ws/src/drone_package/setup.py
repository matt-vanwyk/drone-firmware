from setuptools import find_packages, setup

package_name = 'drone_package'

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
    maintainer='spiderweb',
    maintainer_email='matt.vanwyk2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mavsdk_node = drone_package.mavsdk_node:main',
            'drone_state_machine_node = drone_package.drone_state_machine_node:main',
        ],
    },
)
