from setuptools import find_packages, setup

package_name = 'inspire_hand_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simzx',
    maintainer_email='xzhongs1127@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dds_publisher_node = inspire_hand_ros.dds_publisher_node:main',
            'inspire_hand_visualizer_node = inspire_hand_ros.inspire_hand_visualizer_node:main',
            'headless_driver_node = inspire_hand_ros.headless_driver_node:main',
            'finger_control_node = inspire_hand_ros.finger_control_node:main'
            
        ],
    },
)
