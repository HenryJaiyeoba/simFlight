"""
Setup script for the simFlight package.
"""

from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="simflight",
    version="0.1.0",
    author="simFlight Team",
    author_email="henryjaiyeoba@gmail.com",
    description="An educational Python library for aircraft autopilot control systems",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/henryjaiyeoba/simflight",
    packages=find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Topic :: Education",
        "Topic :: Scientific/Engineering :: Physics",
        "Topic :: Scientific/Engineering :: Visualization",
    ],
    python_requires=">=3.8",
    install_requires=[
        "numpy>=1.20.0",
        "matplotlib>=3.4.0",
        "plotly>=5.0.0",
        "jupyter>=1.0.0",
    ],
    extras_require={
        "dev": [
            "pytest>=6.0.0",
            "flake8>=3.9.0",
            "black>=21.5b2",
        ],
        "docs": [
            "sphinx>=4.0.0",
            "sphinx-rtd-theme>=0.5.2",
        ],
    },
)
