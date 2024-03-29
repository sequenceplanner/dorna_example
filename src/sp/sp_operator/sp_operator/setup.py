import os
from glob import glob
from setuptools import setup

package_name = 'sp_operator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
	    (os.path.join('share', package_name), glob('launch/*.launch.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='kristofer',
    author_email='',
    maintainer='kristofer',
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
        	'sp_operator = sp_operator.main:main',
        ],
    },
)
