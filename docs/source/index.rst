.. OpenSoT documentation master file, created by
   sphinx-quickstart on Wed Aug  9 17:38:48 2023.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.
   
Welcome to the OpenSoT's documentation!
=======================================

An Open Source Task Solving library with Constraints

**OpenSoT** is a C++ library designed to streamline the process of writing and solving Quadratic and Linear Programming (QP/LP) problems with a focus on robotic applications, such as Differential Inverse Kinematics (DIK), Inverse Dynamics (ID), and contact force distribution.

Main features:
--------------
- **Robot-Agnostic:** support to  generic fixed/floating-base systems
- **Efficient:** based on `Eigen <https://eigen.tuxfamily.org/index.php?title=Main_Page>`_  for fast and real-time computation
- **Ready & Easy to Use:** An out-of-the-box library of *Tasks* and *Constraints* to create complex control problems, all of which can be efficiently solved using dedicated *Solvers*
- **Easy to Extend:** A C++ API permits to implement new *Tasks*, *Constraints*, and *Solvers*

 



.. toctree::
   :maxdepth: 2
   :caption: Contents:

   intro
   task
   constraint
   stack
   solver
   variables
   aknowledge
   api/library_root



.. Indices and tables
.. ==================

.. * :ref:`genindex`
.. * :ref:`modindex`
.. * :ref:`search`



