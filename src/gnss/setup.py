from setuptools import find_packages, setup

package_name = 'gnss'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pynmea2'],
    zip_safe=True,
    maintainer='bernhard',
    maintainer_email='bernhard-hoerl@gmx.at',
    description='Integrates A7670E GNSS sensor into ros.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gnss_node = gnss.gnss_node:main',
        ],
    },
)
