from setuptools import setup

setup(
    name="myrmidon",
    version="1.0",
    description="A python package to remote control ground-based agents",
    author="Atharv Marathe",
    author_email="amarathe9@gatech.edu",
    packages=["myrmidon"],
    install_requires=["numpy", "pynput", "matplotlib", "robotarium-python-simulator"],
)
