from setuptools import find_packages, setup

package_name = 'force_bar'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bernhard',
    maintainer_email='bernhard-hoerl@gmx.at',
    description='Merger force bar gazebo topics into one.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'force_bar_node = force_bar.force_bar_node:main',
        ],
    },
)
