# 사용법: setup.py (옵션 - 패키지로 설치하고 싶은 경우)

from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="mujoco-tidybot-simulation",
    version="1.0.0",
    author="MuJoCo Tidybot Team",
    description="Autonomous navigation and manipulation simulation for Stanford Tidybot",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/YOUR_USERNAME/mujoco-tidybot-simulation",
    packages=find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Topic :: Scientific/Engineering :: Robotics",
        "Topic :: Scientific/Engineering :: Physics",
    ],
    python_requires=">=3.8",
    install_requires=[
        "mujoco>=2.3.0",
        "numpy>=1.20.0",
        "scipy>=1.7.0",
        "pynput>=1.7.0",
        "ruckig>=0.9.0",
        "matplotlib>=3.5.0",
    ],
    entry_points={
        "console_scripts": [
            "tidybot-sim=main:main",
            "tidybot-mapping=test_lidar_interactive:main",
        ],
    },
)
