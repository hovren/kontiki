#################
Development tools
#################

Useful information when hacking on Kontiki

Developing with CLion
=====================
#. Create an appropriate conda environment
#. Under ``Settings > Build [...] > Python Interpreter`` find the
   python binary for that environment.
#. Create a run/debug configuration of type ``Python`` and make it run ``setup.py develop``.
   Set working directory to project root.
#. Create a run/debug configuration of type ``Python Tests > py.test`` that runs
   the tests in ``tests/``.
   Under `Before launch` add the ``setup.py develop`` configuration.
