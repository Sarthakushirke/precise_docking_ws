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
            "frontier_detection = active_localization.frontier_detection:main"
        ],
    },
)
