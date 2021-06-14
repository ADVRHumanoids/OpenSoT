soth
====

[![Building Status](https://travis-ci.org/stack-of-tasks/soth.svg?branch=master)](https://travis-ci.org/stack-of-tasks/soth)
[![Pipeline status](https://gepgitlab.laas.fr/stack-of-tasks/soth/badges/master/pipeline.svg)](https://gepgitlab.laas.fr/stack-of-tasks/soth/commits/master)
[![Coverage report](https://gepgitlab.laas.fr/stack-of-tasks/soth/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/stack-of-tasks/doc/stack-of-tasks/soth/master/coverage/)


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
