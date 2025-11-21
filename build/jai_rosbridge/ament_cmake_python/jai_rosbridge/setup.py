from setuptools import find_packages
from setuptools import setup

setup(
    name='jai_rosbridge',
    version='0.0.0',
    packages=find_packages(
        include=('jai_rosbridge', 'jai_rosbridge.*')),
)
