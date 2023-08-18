Solver
======
A **solver** implements a mathematical method to resolve a QP/LP problem (a *stack*).
A *solver* is implemented deriving from the ``OpenSoT::Solver`` class in `Solver.h <https://advrhumanoids.github.io/OpenSoT/api/classOpenSoT_1_1Solver.html>`__ implementing the virtual method ``solve(Vector_type& solution)``.

eHQP
----
The ``eHQP`` solver (`eHQP.h <https://advrhumanoids.github.io/OpenSoT/api/classOpenSoT_1_1solvers_1_1eHQP.html>`__) is based on the work from *Flacco et. al*: *Prioritized Multi-Task Motion Control of Redundant Robots under Hard Joint Constraints* (`paper <https://khatib.stanford.edu/publications/pdfs/Flacco_2012.pdf>`__) and permits to resolve QP problem with **equality only constraints**, without considering the :math:`\mathbf{c}` term in the cost function. Singularities are checked and avoided thorugh damped pseudo-inverse. Solution of the problem is provided by the use of ``Eigen::JacobiSVD`` (`webpage <https://eigen.tuxfamily.org/dox/classEigen_1_1JacobiSVD.html>`__).

The eHQP solver permits to set mulitple levels of *hard* priorities between tasks. When two or more tasks are at a *soft* priority, they are intended to be summed in the same cost function with a different weight, for example, given two tasks :math:`\mathcal{T}_1` and :math:`\mathcal{T}_2`, is possible to write a cost function where both the tasks are solved together but with :math:`\mathcal{T}_1` which is 10 times more important than :math:`\mathcal{T}_2`:

.. math::

   \mathcal{T}_3 = \mathcal{T}_1 + 0.1*\mathcal{T}_2
   
meaning that when the solver will give more importance (ten times) to :math:`\mathcal{T}_1` w.r.t. :math:`\mathcal{T}_2`. However, if the two tasks are minimizing different quantities, they may not be scaled the same, leading to scaling issues (e.g. position in meters and orientation in radiants). Therefore, the weight *0.1* may not be enough to properly set a *soft* priority between the tasks. 

In these cases is possible to set a **hard** priority, meaning that the highest priority task will be minimized regardless the less priority task. There are several techniques to enforce hard priority between tasks, for example *null-space projection*, which is implemented in the ``eHQP`` solver. To enforce hard-priorities between tasks using the *MoT* you just need to use the ``/`` operator:

.. math::

   \mathcal{S}_1 = \mathcal{T}_1 / \mathcal{T}_2 
   
where :math:`\mathcal{S}_1` is a *stack*. **Notice that the way the** *hard* **priority is implemented depends on the particular** *solver* **used**. *Soft* and *hard* priorities can be mixed together and several layers can be created up to saturate the Degrees of Freedom (DoFs) of the problem:

.. math::

   \mathcal{S}_1 = \mathcal{T}_1 / (\mathcal{T}_2+\mathcal{T}_3) / \mathcal{T}_4

