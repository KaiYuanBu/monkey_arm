import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
  	    (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
  	    (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kean Hao',
    maintainer_email='monkey@todo.todo',
    description='Control interface for MonKey\'s Arm',
    license='TODO: License declaration',
    tests_require=['pytest'],
    
    entry_points={
        'console_scripts': [
            'arm_rviz_publisher = arm_control.arm_rviz_publisher:main',
            'bearing_interface = arm_control.bearing_interface:main',
            'behaviour_tree = arm_control.behaviour_tree:main',
            'cylinder_interface = arm_control.cylinder_interface:main',
            'inverse_kinematics = arm_control.inverse_kinematics:main',
        ],
    },
)
