from setuptools import setup, find_packages

setup(
    name="Hopper-CVXPY-Trajectory-Generator",
    version="0.0.1",
    description="Trajectory generator using CVXPY and CVXPYgen",
    author="Jannis Malkomes",
    packages=find_packages(include=["src"]),
    install_requires=[
        "cvxpy",
        "cvxpygen"
    ]
)
