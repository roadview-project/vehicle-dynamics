from setuptools import setup, find_packages

VERSION = '1.0.6' 
DESCRIPTION = 'vehicle_dynamics'
LONG_DESCRIPTION = 'package that calculates the vehicle dynamics'

# Setting up
setup(
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
    ],

    keywords=['python', 'vehicle dynamics'],
    classifiers= [
        "Programming Language :: Python :: 3",
    ]
)
