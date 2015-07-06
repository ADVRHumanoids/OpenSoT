QP based Solver
===============

This solver is based on a cascade of Quadratic Programming (QP) optimization problems. Each problem is solved using
a QP solver called [qpOASES](https://projects.coin-or.org/qpOASES).

Despite the comments in the code, there are some internal mechanism (or state machines) that could be not clear at 
first sight. 

The QP solver is based on two main classes: <em>QPOasesProblem</em> and <em>QPOases_sot</em>. 

QPOasesProblem:
---------------
This class wrap around qpOASES methods some functionalities that we would like to have in our single-stack IK solver.
When constructed, the QPOasesProblem class instantiates a SQProblem (from qpOases) setting some default options.

Afterwards, the problem is initialized with the following state machine:

![QPOasesProblem::initProblem()](https://github.com/robotology-playground/OpenSoT/blob/devel/doc/QPOasesProblem.init.png)

that return true if the problem is correctly initialized.

Once the problem is initialized is possible to <em>update</em> or <em>add</em> tasks, constraints and bounds. 
In the <em>update</em>, change of size of matrices/vectors is allowed. When the size of Task or Constraint change, a new initialization of the problem
is performed. 
When adding a task, it has to be the same number of cols of the contained task; the new task will be piled to the previous one. Same happens for the constraints and bounds. Remember that when adding bounds, the final number of them has to be equal to the number of variables in the problem. At the end of every <em>add</em> operation, a new initialization of the problem is performed with the new task, constraints and/or bounds.

When the problem is solved, the following state machine is used:

![QPOasesProblem::solve()](https://github.com/robotology-playground/OpenSoT/blob/devel/doc/QPOasesProblem.solve.png)

In the solve, the first solution is attempted using the <em>hotstart</em> functionality of qpOASES. If it fails, a second soluton is attempted with the initialization with initial guess given from the previous bounds (aka <em>warmstart</em>), constraints and solution. If also this fails the init is called as last attempt.

QPOases_sot:
------------
This class implements the state machine dedicated to solve the Stack of Tasks. There are some important aspects to be noticed: first the solver takes a vector of tasks and a list of bounds. The assumption done here is that each task in the vector may contains or not some constraints. These constraints are NOT passed to the following task. This means that, if a constraint is present in two different tasks at a differen level in the stack, it has to be added explicitely before the creation of the solver. Second, the bounds are applied to ALL the stacks since they regards directly the variables of all the problems. We consider also the global constraints that are added to all the stacks. These global constraints are passed directly to the solver as for the bounds.

The stack is created and initialized in the constructor and if something goes wrong, an exception is thrown. 

When solved, the following state machine is used:

![QPOases_sot::solve()](https://github.com/robotology-playground/OpenSoT/blob/devel/doc/QPOases_sot.solve.png)

<em>Optimality</em> and <em>Cost Function</em> depends on the type of control. 
