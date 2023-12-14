iHQP: back-end comparison
=========================
We randomly selected 30 IK problems, each involving a transition from an initial configuration to a goal configuration defined by a specific Cartesian pose. 
Each IK problem was solved using the iHQP, hHQP, and HCOD front-end solvers. In particular, for the iHQP and nHQP solvers, all the back-ends were included in the comparison: *OSQP*, *qpOASES*, *eigQuadProg*, *proxQP*, and *qpSWIFT*.

We compared the average time taken to solve a single instance of a hierarchical problem, including the time needed to compute the Hessian and gradient, as well as to fill the solver matrices. 
Additionally, we calculated the success rate :math:`SR`, defined as the number of times the back-end successfully found a solution within certain conditions, divided by the total number of runs:

.. math::

   SR = \frac{\#success}{\#runs}

To confirm that the back-end successfully performs the task, we consider the task error norm to be less than or equal to :math:`1e^{-3}` within the specified maximum number of iterations, set to 1000. Local minima as well near singular configurations may reduce the SR. 

We performed the comparison considering two prfoundly diverse robotic systems: a 7 DOFs fixed-base manipulator and a 29+6 DOFs floating-base humanoids. For each system we consider different stacks characterized by different levels of hard priorities.
The comparison was conducted on an AMD® Ryzen 9 4900HS with 32 GiB of RAM.

.. warning::

	The tests were conducted without fully leveraging the extensive capabilities of the examined QP solvers, both in terms of tunable options and optimal implementation. For example, some of the QP solvers only offer a sparse interface, necessitating the conversion of dense matrices, which are typical of the problems in the OpenSoT framework, into sparse format. Consequently, the obtained results may be affected by suboptimal option tuning and less-than-optimal implementation. It's important to note that these results are provided solely to offer a preliminary understanding of the performance in solving specific control problems and may vary in different runs.
	
.. note::

	A great comparison of QP solvers can be found at the following `GitHub repository <https://github.com/qpsolvers/qpbenchmark/>`__ (by Stéphane Caron). 

Manipulator (7 DOFs)
--------------------
For the manipulator we consider just an end-effector positioning task :math:`\in \mathbb{R}^3`, meaning that the the null-space size is 4.
We consider the following two stacks:

- **Stack 1**: :math:`(\mathcal{T}_{ee}\%\{0,1,2\} + 1e^{-4}\cdot\mathcal{T}_{p})<<\mathcal{C}_{jl}<<\mathcal{C}_{vl}`, only *soft* priorities;
- **Stack 2**: :math:`(\mathcal{T}_{ee}\%\{0,1,2\} / \mathcal{T}_{p})<<\mathcal{C}_{jl}<<\mathcal{C}_{vl}`, 2 levels of *hard* priorites;

with :math:`\mathcal{T}_{ee}` the Cartesian task associated to the end-effector, :math:`\mathcal{T}_{p}` the joint-space postural task, :math:`\mathcal{C}_{jl}` the joint-limits constraint, :math:`\mathcal{C}_{vl}` the velocity limit constraint. Notice that from the Cartesian task, only the SubTask with the position part is considered.  

The statistics comparing the performance are presented in `this figure <_static/panda_plot_ik.pdf>`_. We excluded the HCOD solver from the comparison as it performed poorly in terms of solving time compared to other implementations. For robots with few DOFs, simple constrained IK problems can be solved very fast, under  :math:`0.1 \ [ms]` with a success rate higher than 90%.

Humanoid (35 DOFs)
------------------
For the humanoid we consider Cartesian pose tasks :math:`\in SE(3)` to model the feet in contact with the environment and the tasks at the left and right hands. We consider a quati-static condition of stability by controlling the position :math:`\in \mathbb{R}^3` Center of Mass (CoM) of the robot. All these tasks together occupies 27 of the 29 DOFs available in the robot, resulting in a null-space of size 2.

We consider the following four stacks:

- **Stack 1**: :math:`(1e^{-1}\cdot\mathcal{T}_{lh} + \mathcal{T}_{rh} + \mathcal{T}_{CoM} + 1e^{-4}\cdot\mathcal{T}_{p})<<\mathcal{C}_{jl}<<\mathcal{C}_{vl}<<(\mathcal{T}_{ls} + \mathcal{T}_{rs})`, only *soft* priorities;
- **Stack 2**: :math:`((1e^{-1}\cdot\mathcal{T}_{lh} + \mathcal{T}_{rh} + \mathcal{T}_{CoM}) / \mathcal{T}_{p})<<\mathcal{C}_{jl}<<\mathcal{C}_{vl}<<(\mathcal{T}_{ls} + \mathcal{T}_{rs})`, 2 levels of *hard* priorites. The first level has *soft* priorities;
- **Stack 3**: :math:`(\mathcal{T}_{CoM} / (1e^{-1}\cdot\mathcal{T}_{lh} + \mathcal{T}_{rh}) / \mathcal{T}_{p})<<\mathcal{C}_{jl}<<\mathcal{C}_{vl}<<(\mathcal{T}_{ls} + \mathcal{T}_{rs})`, 3 levels of *hard* priorities. The second level has *soft* priorities;
- **Stack 4**: :math:`(\mathcal{T}_{CoM} / \mathcal{T}_{rh} / \mathcal{T}_{lh} / \mathcal{T}_{p})<<\mathcal{C}_{jl}<<\mathcal{C}_{vl}<<(\mathcal{T}_{ls} + \mathcal{T}_{rs})`, 4 levels of *hard* priorities;

with :math:`\mathcal{T}_{lh}` and :math:`\mathcal{T}_{rh}` the Cartesian task associated to the left and right hands respectively, :math:`\mathcal{T}_{lf}` and :math:`\mathcal{T}_{rf}` the Cartesian task associated to the left and right feet respectively, and :math:`\mathcal{T}_{CoM}` the CoM task. Notice that the feet tasks are used as constraints in these stacks. The desired Cartesian references are sent to the :math:`\mathcal{T}_{rh}` task, while to the :math:`\mathcal{T}_{lh}` is requested to keep its pose with less priority.  

The statistics comparing the performance are presented in `this figure <_static/coman_plot_ik.pdf>`_. We again excluded the HCOD solver from the comparison as it performed poorly in terms of solving time compared to other implementations, although it achieved a 100% success rate. For robots with multiple DOFs, simple constrained IK problems can be solved also fast, within :math:`1 \ [ms]` and :math:`10 \ [ms]`. The success rate is for most of the cases between 80% and 100%. Notably, in the case of the iHQP, the time taken to resolve multiple layers does not change significantly when increasing the number of layers to values typically used in complex robotics systems (4-5 layers).



   


