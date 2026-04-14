from setuptools import find_packages, setup

package_name = 'brain_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python', 'ultralytics'],
    zip_safe=True,
    maintainer='tdun',
    maintainer_email='tiendungbnts@gmail.com',
    description='Brain package for Autonomous Robot (Vision & Navigation)',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision = brain_pkg.vision_node:main',
            'navigation = brain_pkg.navigation_node:main'
        ],
    },
)
