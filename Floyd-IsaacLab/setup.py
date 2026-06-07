from setuptools import setup, find_packages

setup(
    name="BDXR",
    version="0.1.0",
    packages=find_packages(),
    author="Logan",
    description="Floyd BDX-style biped Isaac Lab training config",
    python_requires=">=3.10",
    install_requires=["psutil"],
)
