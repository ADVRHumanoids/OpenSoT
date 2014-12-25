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

Afterwards, the problem is initialized with the following state machine:

![QPOasesProblem::initProblem()](https://github.com/robotology-playground/OpenSoT/blob/qp_solver/doc/QPOasesProblem.init.png)

that return true if the problem is correctly initialized.

Once the problem is initialized is possible to <em>update</em> or <em>add</em> tasks, constraints and bounds. 
In the <em>update</em>, change of size of matrices/vectors is allowed. When the size of Task or Constraint change, a new initialization of the problem
is performed. 
When adding a task, it has to be the same number of cols of the contained task; the new task will be piled to the previous one. Same happens for the constraints and bounds. Remember that when adding bounds, the final number of them has to be equal to the number of variables in the problem. At the end of every <em>add</em> operation, a new initialization of the problem is performed with the new task, constraints and/or bounds.

When the problem is solved, the following state machine is used:

![QPOasesProblem::solve()](https://github.com/robotology-playground/OpenSoT/blob/qp_solver/doc/QPOasesProblem.solve.png)

In the solve, the first solution is attempted using the <em>hotstart</em> functionality of qpOASES. If it fails, a second soluton is attempted with the initialization with initial guess given from the previous bounds, constraints and solution. If also this fails the init is called as last attempt.




