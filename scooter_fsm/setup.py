from setuptools import setup

package_name = 'scooter_fsm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'scooter_fsm = scooter_fsm.scooter_fsm:main',
            'test_wait_for_begin = scooter_fsm.test_wait_for_begin:main'
        ],
    },
)
