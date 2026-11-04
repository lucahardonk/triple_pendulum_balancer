from setuptools import find_packages, setup

package_name = 'odrive_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml', 'README.md'],
        ),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lucahardonk',
    maintainer_email='YOUR_EMAIL@example.com',
    description='ROS 2 ODrive USB driver with SI units and serialized USB access',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'driver_node = odrive_driver.driver_node:main',
        ],
    },
)