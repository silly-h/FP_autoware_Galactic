from setuptools import setup, find_packages
import glob
import os

package_name = 'can_communication'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['can_communication', 'can_communication.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob.glob(os.path.join('msg', '*.msg'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orin',
    maintainer_email='orin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can_output = can_communication.scripts.can_output:main',
            'pub_twist = can_communication.scripts.pub_twist:main',
        ],
    },
)
