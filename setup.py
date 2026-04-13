from setuptools import setup, find_packages
from pathlib import Path
this_directory = Path(__file__).parent
long_description = (this_directory / "readme.md").read_text()

setup(
    name='robowrap',
    long_description=long_description,
    long_description_content_type='text/markdown',
    version='0.8.0',
    author='Martin Davies',
    author_email='mdavies@kellettschool.com',
    description='A student-first wrapper around the DJI RoboMaster EP Python SDK.',
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
