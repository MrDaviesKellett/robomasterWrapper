from setuptools import setup, find_packages
setup(
    name='robowrap',
    version='0.7.0',
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
    install_requires=[line.strip() for line in open("requirements.txt").readlines()],
)