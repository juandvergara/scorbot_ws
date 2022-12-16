from setuptools import setup

package_name = 'serial_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juanmadrid',
    maintainer_email='juan_dav.vergara@uao.edu.co',
    description='Publisher node to targets robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_publisher = serial_publisher.serial_publisher:main'
        ],
    },
)
