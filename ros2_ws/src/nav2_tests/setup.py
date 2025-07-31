from setuptools import find_packages, setup
from glob import glob

package_name = 'nav2_tests'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')), 

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='avlab_mz',
    maintainer_email='muradsmebrahtu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           'initial_pose_publisher = nav2_tests.initial_pose_publisher:main',
           'nav2_test_action_server = nav2_tests.nav2_test_action_server:main',
        ],
    },
)
