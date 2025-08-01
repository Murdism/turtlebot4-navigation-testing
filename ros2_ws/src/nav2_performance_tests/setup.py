from setuptools import find_packages, setup
from glob import glob

package_name = 'nav2_performance_tests'

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
    maintainer='Murdism',
    maintainer_email='muradsmebrahtu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           'amcl_pose_initializer = nav2_performance_tests.amcl_pose_initializer:main',
           'Nav2TestNode = nav2_performance_tests.Nav2TestNode:main',
        ],       
    },

    py_modules=[
        'nav2_performance_tests.navigation_test_error_handler',
        'nav2_performance_tests.Nav2TestNode',
        'nav2_performance_tests.amcl_pose_initializer',
    ],
)
