from setuptools import find_packages
from setuptools import setup

setup(
    name='pickplace_rl_mobile',
    version='0.0.1',
    packages=find_packages(
        include=('pickplace_rl_mobile', 'pickplace_rl_mobile.*')),
)
