from setuptools import setup

package_name = 'src_gazebo_controller'

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
    maintainer='kimsooyoung',
    maintainer_email='tge1375@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'src_gazebo_controller = src_gazebo_controller.src_gazebo_controller:main',
            'odom_utility_tools = src_gazebo_controller.odom_utility_tools:main',
            'basic_control = src_gazebo_controller.basic_control:main',
        ],
    },
)
