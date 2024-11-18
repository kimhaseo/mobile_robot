from setuptools import setup
from glob import glob

package_name = 'mobile_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/description', glob('description/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='has',
    maintainer_email='has@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_publisher = mobile_robot.odom_publisher:main',
        ],
    },
)
