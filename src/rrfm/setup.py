from setuptools import find_packages, setup

package_name = 'rrfm'

setup(
    name=package_name,
    version='0.1.0',
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
    description='Random Robot Forest Motion',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rrfm_node = rrfm.rrfm_node:main',
        ],
    },
)
