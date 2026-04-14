import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'cvrp_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/maps', glob('cvrp_planner/maps/*')),
        ('share/' + package_name + '/params', glob('cvrp_planner/params/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (
            os.path.join('share', package_name, 'isaac'),
            glob('scripts/isaac/*.py')
            + glob('scripts/isaac/*.json')
            + glob('scripts/isaac/*.sh')
            + glob('scripts/isaac/*.md'),
        ),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='parallels',
    maintainer_email='2579947814@qq.com',
    description='Multi-robot CVRP task planner for Isaac Sim and Nav2.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
            'cvrp_node = cvrp_planner.cvrp_node:main',
            'params_generator = cvrp_planner.params_generator:main',
            'xml_splitter = cvrp_planner.xml_splitter:main',
            'odom_tf_bridge = cvrp_planner.odom_tf_bridge:main',

        ],
    },
)
