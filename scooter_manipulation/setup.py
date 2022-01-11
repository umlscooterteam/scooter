from setuptools import setup
import os
from glob import glob

package_name = 'scooter_manipulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sasha',
    maintainer_email='sashaiw@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick = scooter_manipulation.pick:main',
            'static_collision_object_publisher = scooter_manipulation.static_collision_object_publisher:main'
        ],
    },
)
