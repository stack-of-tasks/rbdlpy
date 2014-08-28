rbdlpy
===========

Setup
-----

To compile this package, it is recommended to create a separate build
directory:

    mkdir _build
    cd _build
    cmake [OPTIONS] ..
    make install

Please note that CMake produces a `CMakeCache.txt` file which should
be deleted to reconfigure a package from scratch.


### Dependencies

The matrix abstract layer depends on several packages which
have to be available on your machine.

 - Libraries:
   - eigen3
 - System tools:
   - CMake (>=2.6)
   - pkg-config
   - usual compilation tools (GCC/G++, make, etc.)
 - Python 2.7
 - Boost python

### Unit tests

In the /unittest folder, one can find unit tests for each module of RBDL. For example, kinematics.py demonstrates the functionality of Kinematics module of RBDL using Python. 

### Tutorials

In the /python folder, there are certain tutorials to demonstrate the ability of RBDLPY. For example, FDTest.py demonstrate how to simulate the forward dynamics of a double pendulum system defined using Python. The tutorials are self explanatory and commented properly. 




