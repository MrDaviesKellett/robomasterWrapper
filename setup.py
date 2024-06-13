from setuptools import setup, find_packages
from pathlib import Path
this_directory = Path(__file__).parent
long_description = (this_directory / "README.md").read_text()

setup(
    name='robowrap',
    long_description=long_description,
    long_description_content_type='text/markdown',
    version='0.7.11',
    author='Martin Davies',
    author_email='mdavies@kellettschool.com',
    description='A wrapper for the DJI robomaster library for Python, makes programming the robomaster a little eeasier.',
    packages=find_packages(),
    classifiers=[
    'Programming Language :: Python :: 3',
    'License :: OSI Approved :: MIT License',
    'Operating System :: OS Independent',
    ],
    python_requires='>=3.7, <3.9',
    install_requires=['robomaster',
                      'simple_pid',
                      'opencv-python==4.2.0.34 ; platform_system=="Windows"',
                      'opencv-python ; platform_system!="Windows"'],
)