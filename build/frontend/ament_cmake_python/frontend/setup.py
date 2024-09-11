from setuptools import find_packages
from setuptools import setup

setup(
    name='frontend',
    version='0.0.0',
    packages=find_packages(
        include=('frontend', 'frontend.*')),
)
