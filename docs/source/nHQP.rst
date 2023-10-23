nHQP
----
The ``nHQP`` solver (`nHQP.h <https://advrhumanoids.github.io/OpenSoT/api/classOpenSoT_1_1solvers_1_1nHQP.html>`__) is based on the work from *authors*: *de Lasa et al.* (`Prioritized Optimization for Task-Space Control <https://ieeexplore.ieee.org/abstract/document/5354341>`__) and permits to resolve hierarchical QP problems with **generic inequality constraints**.

As in the case of the iHQP solver, hierarchies are solved by a cascade of QP problems, but in this case, the null-space of high priority tasks is exploited to solve only the remaining available free DOFs, resulting in successive optimizations with reduced number of variables.  

The generic *k-th* priority is implemented as:

.. math::

	\begin{align}
   		&\mathbf{\bar{x}}_k^* = arg\min\limits_{\mathbf{\bar{x}}_k} \lVert \mathbf{\bar{A}}_k\mathbf{\bar{x}}_k - \mathbf{\bar{b}}_k\rVert \newline
   		&s.t. \quad  \mathbf{\bar{d}}_{m,k}\leq \mathbf{\bar{C}}_k\mathbf{x}\leq\mathbf{\bar{d}}_{M,k} 
   	\end{align}

with: 

.. math::

	\begin{align}
	\mathbf{\bar{A}}_k &= \mathbf{A}_k\mathbf{N}_{k-1}, \\
	\mathbf{\bar{b}}_k &= \mathbf{b}_k-\mathbf{A}_k\mathbf{x}_{k-1}, \\
	\mathbf{\bar{C}}_k &= \mathbf{A}_k\mathbf{N}_{k-1}, \\
	[\mathbf{\bar{d}}_{m,k}, \mathbf{\bar{d}}_{M,k}] &= [\mathbf{d}_{m,k}-\mathbf{A}_k\mathbf{x}_{k-1}, \mathbf{d}_{M,k}-\mathbf{A}_k\mathbf{x}_{k-1}]
	\end{align} 
	
and :math:`\mathbf{N}_{k-1}` is a *cumulative* base of the null-space of the previous *k-1* levels computed as: 

.. math::

	\mathbf{N}_{k-1} = \mathbf{N}_{k-2} * Null(\mathbf{A}_{k-1}\mathbf{N}_{k-2}). 
	
The final solution is computed ad :math:`\mathbf{x}_k = \mathbf{x}_{k-1} + \mathbf{N}_{k-1}*\mathbf{\bar{x}}_k^*`.

For the first priority level :math:`k = 0`:
 
 .. math::
 
 	\begin{align}
 	\mathbf{N}_{0} &= \mathbf{I}, \\
 	\mathbf{x}_{0} &= \mathbf{0}, \\
 	\mathbf{N}_{1} &= Null(\mathbf{A}_1).
 	\end{align}
 	
Notice that :math:`\mathbf{x}_k \in \mathbb{R}^p`, :math:`\mathbf{x}_{k-1} \in \mathbb{R}^l`, ..., :math:`\mathbf{x}_1 \in \mathbb{R}^n`, with :math:`p < l < ... < n`. 
:math:`Null(\mathbf{H})` can be computed thorugh SVD; in particular we use the `Bidiagonal Divide and Conquer SVD <https://eigen.tuxfamily.org/dox/classEigen_1_1BDCSVD.html>`__ provided by Eigen. 

The *back-ends* available for the nHQP *front-end* are:

- `qpOASES <https://github.com/coin-or/qpOASES>`__ v3.1 internally distributed
- `eiQuadProg <https://www.cs.cmu.edu/~bstephe1/eiquadprog.hpp>`__ v1.0.0 internally distributed
- `OSQP <https://osqp.org/>`__ suggested Release 0.6.2
- `proxQP <https://github.com/Simple-Robotics/proxsuite>`__ suggested v0.1.0
- `qpSWIFT <https://github.com/qpSWIFT/qpSWIFT>`__ suggested v1.00

It is worth noticing that the computation of the SVD permits to access to information on the rank of the Task matrix with the possiblity to apply some strategy to handle singularities. In particular, we added regularization to the :math:`\mathbf{A}` matrix and :math:`\mathbf{b}` vector based on the singular Cartesian directions, and to the Hessian matrix based on the null-space.  
