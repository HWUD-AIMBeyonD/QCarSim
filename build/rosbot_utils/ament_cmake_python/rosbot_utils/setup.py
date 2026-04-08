from setuptools import find_packages
from setuptools import setup

setup(
    name='rosbot_utils',
    version='0.18.8',
    packages=find_packages(
        include=('rosbot_utils', 'rosbot_utils.*')),
)
