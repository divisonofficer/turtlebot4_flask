from setuptools import find_packages
from setuptools import setup

setup(
    name='dcs103e_controller',
    version='0.0.1',
    packages=find_packages(
        include=('dcs103e_controller', 'dcs103e_controller.*')),
)
