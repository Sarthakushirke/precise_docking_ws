from setuptools import find_packages, setup

package_name = 'active_localization'

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
    maintainer='sarthak',
    maintainer_email='37041521+Sarthakushirke@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "local_map_node = active_localization.local_map:main",
            "frontier_detection = active_localization.frontier_detection:main",
            "path_to_goal = active_localization.path_to_goal:main",
            "localization = active_localization.localization:main",
            "local_map_lidar_node = active_localization.local_map_lidar:main",
            "local_map_robot = active_localization.local_map_robot:main",
            "local_map_robot_1 = active_localization.local_map_robot_1:main",
            "frontier_detection_robot = active_localization.frontier_detection_robot:main"
        ],
    },
)
