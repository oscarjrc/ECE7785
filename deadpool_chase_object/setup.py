import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'deadpool_chase_object'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oscar',
    maintainer_email='oscar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'detectobj = deadpool_chase_object.detect_object:main',
        'viewimg = deadpool_chase_object.view_image:main',
        'getobjrng = deadpool_chase_object.get_object_range:main',
        'chaseobj = deadpool_chase_object.chase_object:main',
        ],
    },
)
