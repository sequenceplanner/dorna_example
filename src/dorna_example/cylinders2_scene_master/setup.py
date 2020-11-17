import os
from glob import glob
from setuptools import setup

package_name = 'cylinders2_scene_master'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
	(os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('meshes/*')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='endree',
    author_email='',
    maintainer='endree',
    maintainer_email='',
    keywords=['ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Autogenerated ROS2 package',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'cylinders2_scene_master = cylinders2_scene_master.cylinders2_scene_master:main',
            'add_remove_cube_tester = cylinders2_scene_master.add_remove_cube_tester:main',
            'move_robots_tester = cylinders2_scene_master.move_robots_tester:main',
            'gripping_tester = cylinders2_scene_master.gripping_tester:main',
        ],
    },
)
