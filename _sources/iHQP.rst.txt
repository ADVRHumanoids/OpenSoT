iHQP
----
The ``iHQP`` solver (`iHQP.h <https://advrhumanoids.github.io/OpenSoT/api/classOpenSoT_1_1solvers_1_1iHQP.html>`__) is based on the work from *Kanoun et. al*: *Kinematic control of redundant manipulators: generalizing the task priority framework to inequality tasks* (`paper <https://citeseerx.ist.psu.edu/document?repid=rep1&type=pdf&doi=a98060c46adf364b21f7e197edca2abc774c8c98>`__) and permits to resolve hierarchical QP problems with **generic equality and inequality constraints**. 

Hierarchies are solved by a cascade of QP problems where high-level tasks become constraints in lower level ones. Let's consider for example the following *stack*:

.. math::

   \mathcal{S} = (\mathcal{T}_1 / \mathcal{T}_2)<<\mathcal{C}_1
   
This correspond to the following cascade of QPs:

1. a QP solving :math:`\mathcal{T}_1<<\mathcal{C}_1` and computing the higher priority solution ::math::`\mathbf{x}_1`:

.. math:: 
   
   \begin{align}
   &\mathbf{x}_1 = arg\min\limits_{\mathbf{x}} \lVert \mathbf{A}_1\mathbf{x} - \mathbf{b}_1\rVert \newline
   &s.t. \quad  \mathbf{d}_{m,1}\leq \mathbf{C}_1\mathbf{x}\leq\mathbf{d}_{M,1} 
   \end{align} 

2. a QP solving :math:`\mathcal{T}_2<<\mathcal{C}_1<<\mathcal{T}_1^{opt}` and computing the final solution :math:`\mathbf{x}_2 = \mathbf{x}^{opt}`:

.. math:: 
   
   \begin{align}
   &\mathbf{x}_2 = \text{arg}\min\limits_{\mathbf{x}} \lVert \mathbf{A}_2\mathbf{x} - \mathbf{b}_2\rVert \newline
   &s.t. \quad  \mathbf{d}_{m,1}\leq \mathbf{C}_1\mathbf{x}\leq\mathbf{d}_{M,1} \newline
   & \quad \quad \mathbf{A}_1\mathbf{x} = \mathbf{A}_1\mathbf{x}_1 
   \end{align} 

the term :math:`\mathbf{A}_1\mathbf{x} = \mathbf{A}_1\mathbf{x}_1` is called *optimality constraint*.
   
The ``iHQP`` solver consists in a **front-end** (derived from ``OpenSoT::Solver``), that prepares the optimization problems to be solved starting from the given *stack*, and a **back-end** (derived from the ``OpenSoT::solvers::BackEnd`` base class in `BackEnd.h <>`__) which solves the optimization problem using an out-of-the-box QP solver. Different *back-ends* are available in OpenSoT and can be used in different *front-ends*:

- `qpOASES <https://github.com/coin-or/qpOASES>`__ v3.1 internally distributed
- `eiQuadProg <https://www.cs.cmu.edu/~bstephe1/eiquadprog.hpp>`__ v1.0.0 internally distributed
- `GLPK <https://www.gnu.org/software/glpk/>`__ suggested v4.65, only for LP
- `OSQP <https://osqp.org/>`__ suggested Release 0.6.2
- `proxQP <https://github.com/Simple-Robotics/proxsuite>`__ suggested v0.1.0
- `qpSWIFT <https://github.com/qpSWIFT/qpSWIFT>`__ suggested v1.00

.. note::

   Some *front-ends*, e.g. `qpOASES`, permit to solve LP problems, i.e. optimization problems where the :math:`\mathbf{A}` matrix is null (hence also the Hessian is null). **GLPK** is a LP dedicated solver that can not be used to solve QP problems. 
   
The ``iHQP`` constructor permits to decide which *back-end* solver to use, as well as specify a vector of *back-ends*, one per hierarchy level.

Reqularization of the cost function can be added to handle *task* singularities as well as non square problems, i.e. when :math:`\mathbf{A}\in\mathbb{R}^{m \times n}` with :math:`m < n`, through the constructor parameter ``eps_regularisation``. Different values for each level of the stack can be set thorugh the ``setEpsRegularisation(const double eps, const unsigned int i)`` method.

.. note::

   The regularization term can be seen as *task* with :math:`\mathbf{A} = \mathbf{I}\in\mathbb{R}^{n \times n}`,  :math:`\mathbf{b} = \mathbf{0}\in\mathbb{R}^n`, and :math:`\mathbf{W} = \epsilon \in \mathbb{R}`. However the regularization is not considered when computing the optimality constraint of a level *i*, in order to do not saturate the remaining null-space. This may cause instability of the solution at the level *i-1*, which is well-posed at the level *i* due to the presence of the regularization term, and become ill-posed as a constraint at the level *i-1*. This problem can be solved directly acting on the task matrix :math:`\mathbf{A}_i` as done in the :ref:`nHQP:nHQP` solver.  
   
Another way to specify a task that is NOT moved to the next level of priority when computing the *optimality constraint*, hence is used as a regularization, is through the ``setRegularisationTask(OpenSoT::tasks::Aggregated::TaskPtr regularisation_task)`` method of the ``OpenSoT::AutoStack`` class. For example in this way is possible to regularize at the velocity level a task written in acceleration.  

.. warning::

   The ``setRegularisationTask(OpenSoT::tasks::Aggregated::TaskPtr regularisation_task)`` method of the ``OpenSoT::AutoStack`` class is considered only by the ``iHQP`` solver for the moment!ùù
   
The ``setActiveStack(const unsigned int i, const bool flag)`` permits to select a stack level to do not be solved during the next ``solve`` procedure.

The ``setOptions(const unsigned int i, const boost::any &opt)`` permits to set the options of the *back-end* at the *i-th* priority level. ``getBackEnd(const unsigned int i, BackEnd::Ptr& back_end)`` return a *back-end* pointer at the *i-th* level.    
