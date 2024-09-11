from setuptools import find_packages
from setuptools import setup

setup(
    name='norlab_icp_mapper_ros',
    version='2.0.0',
    packages=find_packages(
        include=('norlab_icp_mapper_ros', 'norlab_icp_mapper_ros.*')),
)
