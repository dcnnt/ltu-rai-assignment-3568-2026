from setuptools import find_packages, setup

package_name = 'mission_bridge_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dan',
    maintainer_email='',
    description='Semantic grounding to NAV2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_bridge_node = mission_bridge_pkg.mission_bridge_node:main',
        ],
    },
)
