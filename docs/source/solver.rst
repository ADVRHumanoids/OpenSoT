Solver
======
A **solver** implements a mathematical method to resolve a QP/LP problem (a *stack*).
A *solver* is implemented deriving from the ``OpenSoT::Solver`` class in `Solver.h <https://advrhumanoids.github.io/OpenSoT/api/classOpenSoT_1_1Solver.html>`__ implementing the virtual method ``solve(Vector_type& solution)``.

*Solvers* in OpenSoT library permit to set mulitple levels of *hard* priorities between tasks. When two or more tasks are at a *soft* priority, they are intended to be summed in the same cost function with a different weight, for example, given two tasks :math:`\mathcal{T}_1` and :math:`\mathcal{T}_2`, is possible to write a cost function where both the tasks are solved together but with :math:`\mathcal{T}_1` which is 10 times more important than :math:`\mathcal{T}_2`:

.. math::

   \mathcal{T}_3 = \mathcal{T}_1 + 0.1*\mathcal{T}_2
   
meaning that when the solver will give more importance (ten times) to :math:`\mathcal{T}_1` w.r.t. :math:`\mathcal{T}_2`. However, if the two tasks are minimizing different quantities, they may not be scaled the same, leading to scaling issues (e.g. position in meters and orientation in radiants). Therefore, the weight *0.1* may not be enough to properly set a *soft* priority between the tasks. 

In these cases is possible to set a **hard** priority, meaning that the highest priority task will be minimized regardless the less priority task. There are several techniques to enforce hard priority between tasks, for example *null-space projection*, which is implemented in the :ref:`eHQP:eHQP` solver. To enforce hard-priorities between tasks using the *MoT* you just need to use the ``/`` operator:

.. math::

   \mathcal{S}_1 = \mathcal{T}_1 / \mathcal{T}_2 
   
where :math:`\mathcal{S}_1` is a *stack*. **Notice that the way the** *hard* **priority is implemented depends on the particular** *solver* **used**. *Soft* and *hard* priorities can be mixed together and several layers can be created up to saturate the Degrees of Freedom (DoFs) of the problem:

.. math::

   \mathcal{S}_1 = \mathcal{T}_1 / (\mathcal{T}_2+\mathcal{T}_3) / \mathcal{T}_4
   
Available Solvers
-----------------
OpenSoT provides out-of-the-box *solvers*, in particular:

- The :ref:`eHQP:eHQP` solver for **equality-only Hierarchical QPs**
- The :ref:`iHQP:iHQP` solver for **inequality Hierarchical QPs** 
- The :ref:`nHQP:nHQP` solver for **null-space Hierarchical QPs**
- The :ref:`HCOD:HCOD` solver for **Hierarchical Complete Orthogonal Decomposition**

You can find :doc:`here a comparison <solvers_comparison>` among the different solvers.


