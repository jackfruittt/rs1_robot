from setuptools import find_packages
from setuptools import setup

setup(
    name='rs1_robot',
    version='0.0.1',
    packages=find_packages(
        include=('rs1_robot', 'rs1_robot.*')),
)
