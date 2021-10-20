CMake packages
**************

This modules allow to work with the target-based approach of CMake introduced in CMake 2.8.12.

Generating a CMake package
==========================

Compared to the default minimal project one needs to:

- Specify: ``EXPORT ${TARGETS_EXPORT_NAME}`` for at least one target the
  project should export;

- Call ``SETUP_PROJECT_PACKAGE_FINALIZE()``

The following two conditions will ensure that a correct version, config and targets scripts are generated.

Note that all exported targets will be in the ``${PROJECT_NAME}`` namespace.

Extra information
-----------------

In some situations you might want to add some extra information to the
generated config script. This can be done by manipulating the
``PROJECT_EXTRA_MACROS`` variable. The content of this variable will be
appended to the end of the config script.

Consuming packages
==================

.. cmake-module:: ../../package-config.cmake

Minimal working example with CMake packages
===========================================

.. literalinclude:: ../examples/minimal-with-packages.cmake
  :language: cmake
