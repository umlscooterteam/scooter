import os
from glob import glob
from setuptools import setup

package_name = 'scooter_ui_dummy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sam Laderoute',
    maintainer_email='laderoute.samuel@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'wait_for_begin_service_server = scooter_ui_dummy.wait_for_begin:main',
        	'pick_selection_service_server = scooter_ui_dummy.pick_selection:main',
        ],
    },
)
