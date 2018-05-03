############
Installation
############

Precompiled wheel
=================

We build precompiled binary wheels that should hopefully work on most newer Linux distributions.
Download the ``kontiki-X.Y-cp36-cp36m-linux_x86_64.whl`` file from the GitHub release page.
The run::

    pip install /path/to/kontiki-X.Y-cp36-cp36m-linux_x86_64.whl

Note that this requires that you have the `Ceres-Solver <https://ceres-solver.org>`_ shared library
(``libceres.so.1``), and its dependencies, installed on your system.

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
* `Sophus <https://github.com/strasdat/Sophus>`_  (use commit ``00f3fd91c153ef04``)
* pytest (optional)

To install all dependencies (except Sophus) on Ubuntu 18.04 run::

    apt-get install git build-essential cmake python3-dev python3-setuptools python3-pytest python3-scipy python3-h5py libceres-dev


Instructions
-------------
#. Clone the source code from the repository at `<https://github.com/hovren/kontiki>`_.
#. From ``kontiki/python/`` run ``python3 setup.py install``
