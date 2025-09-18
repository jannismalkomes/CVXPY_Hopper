from setuptools import setup, find_packages

setup(
    name="Base set-up",
    version="0.0.1",
    description="A python module wrapping the Space Team Aachen STAHR mission analysis tools.",
    author="Jannis Malkomes",
    packages=find_packages(include=["mission_analysis"]),
    install_requires=[
        "pandas",
        "numpy",
        "scipy",
        "tabulate"
    ]
)