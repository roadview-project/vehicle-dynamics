from setuptools import setup, find_packages

VERSION = '1.0.0' 
DESCRIPTION = 'vehicle_dynamics'
LONG_DESCRIPTION = 'package that calculates the vehicle dynamics'

# Setting up
setup(
    # the name must match the folder name 'verysimplemodule'
    name="vehicle_dynamics", 
    version=VERSION,
    author="Maikol Funk Dreschler, Yuri Poledna",
    author_email="<Maikol.FunkDreschler@thi.de>",
    description=DESCRIPTION,
    long_description=LONG_DESCRIPTION,
    packages=find_packages(),
    install_requires=[
        'numpy',
        'matplotlib',
        'scipy',
        "pyyaml",
        "tqdm",
        "pandas",
        "munch",
        "urllib",
    ],

    keywords=['python', 'vehicle dynamics'],
    classifiers= [
        "Programming Language :: Python :: 3",
    ]
)
