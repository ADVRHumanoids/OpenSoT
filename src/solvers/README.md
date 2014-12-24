QP based Solver
===============

This solver is based on a cascade of Quadratic Programming (QP) optimization problems. Each problem is solved using
a QP solver called [qpOASES](https://projects.coin-or.org/qpOASES).

Despite the commented code, there are some internal mechanism (or state machines) that could be not clear at 
first sight. 

QPOasesProblem:
---------------
This class wrap around qpOASES methods some functionalities that we would like to have in our single-stack IK solver.
When constructed, the QPOasesProblem class instantiates a SQProblem (from qpOases) setting some default options.

Afterwards the problem is initialized with the following state machine:
