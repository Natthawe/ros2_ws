import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'reconfig'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='natthawe',
    maintainer_email='natthawejumjai@gmail.com',
    description='RECONFIGURE',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'dynamic_parameter_node = reconfig.dynamic_parameter_node:main',
            # 'hello = reconfig.hello_node:main',
            # 'devices = reconfig.dynamic_reconfig:main',
            # 'gui=reconfig.param_reconfig_gui:main',
            'node_reconfig=reconfig.parameter_reconfigure_node:main',
            'gui_reconfig=reconfig.parameter_reconfigure_gui:main',
        ],
    },
)
