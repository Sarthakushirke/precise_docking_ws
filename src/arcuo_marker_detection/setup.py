from setuptools import find_packages, setup

package_name = 'arcuo_marker_detection'

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
            "aruco_marker_detection_node = arcuo_marker_detection.arcuo_detection:main",
            # "aruco_marker_detection = arcuo_marker_detection.pose_arcuo_marker:main"
        ],
    },
)
