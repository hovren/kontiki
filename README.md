Trajectory and Structure estimator
====================================

Developing with CLion
-----------------------
1. Create an appropriate conda environment
2. Under `Settings > Build [...] > Python Interpreter` find the 
python binary for that environment.
3. Create a run/debug configuration of type `Python` and make it run `setup.py develop`.
Set working directory to project root.
4. Create a run/debug configuration of type `Python Tests > py.test` that runs
the tests in `tests/`. 
Under `Before launch` add the `setup.py develop` configuration.
