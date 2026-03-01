from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arm_movement'

def package_files(directory):
    paths = []
    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths


viz_files = package_files('Viz_asset')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        # install launch files
        (os.path.join('share', package_name, 'launch'),
         ['launch/arm_viz.launch.py']),

        #install rviz config file
        (os.path.join('share', package_name, 'resource'),
         glob('resource/*.rviz'))
    ] + [
        # install Viz_asset entire tree
        (os.path.join('share', package_name, os.path.dirname(f)), [f])
        for f in viz_files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kirupa_krishan',
    maintainer_email='kirupa_krishan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'forward_kinematics = arm_movement.forward_kine:main',
            'inverse_kinematics = arm_movement.inv_kine:main',
        ],
    },
)
