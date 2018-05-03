############
Installation
############

Using pip
=========

The package is installable from pip::

    pip install kontiki

Note that this requires that you have `Ceres-Solver <https://ceres-solver.org>` installed on your system.


Using conda
===========
Coming soon.

From source
============

Requirements
------------
* Python 3.6+ and development headers
* A C++14 compiler
* pybind11
* Eigen3
* Ceres-Solver
* CMake
* scipy
* numpy
* h5py
* `Sophus <https://github.com/strasdat/Sophus>`_ (optional)
* pytest (optional)

Ubuntu packages
---------------
git
build-essential
cmake
python3
python3-dev
python3-setuptools
python3-pytest
python3-scipy
python3-h5py
libceres-dev


Instructions
-------------
#. Clone the source code from the repository at `<https://github.com/hovren/kontiki>`_.
#. From ``kontiki/python/`` run ``python3 setup.py install``
