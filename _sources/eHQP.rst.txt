eHQP
----
The ``eHQP`` solver (`eHQP.h <https://advrhumanoids.github.io/OpenSoT/api/classOpenSoT_1_1solvers_1_1eHQP.html>`__) is based on the work from *Flacco et. al*: *Prioritized Multi-Task Motion Control of Redundant Robots under Hard Joint Constraints* (`paper <https://khatib.stanford.edu/publications/pdfs/Flacco_2012.pdf>`__).

The ``eHQP`` solver permits to resolve hierarchical QP problems with **equality only constraints**, without considering the :math:`\mathbf{c}` term in the cost function. Hierarchies are considered though **null-space projection**.

**Singularities are also checked and avoided thorugh damped pseudo-inverse.** Solution of the problem is provided by the use of ``Eigen::JacobiSVD`` (`webpage <https://eigen.tuxfamily.org/dox/classEigen_1_1JacobiSVD.html>`__).


