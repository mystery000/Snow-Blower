from setuptools import find_packages, setup

package_name = 'snow_blower'

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
    maintainer='Mohamed Hafeel',
    maintainer_email='hafeelmo1100@gmail.com',
    description='Snow Blower',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Robot = snow_blower.robot:main'
        ],
    },
)
