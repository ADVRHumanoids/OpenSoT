Constraint
==========
In OpenSoT, **constraints** are utilized to limit a cost function. Considering a specific *linear constraint*, denoted as :math:`\mathcal{C}_1`, it is defined by its corresponding matrices and vectors:

.. math::
  
   \mathcal{C}_1 = \left\{ \mathbf{C}_1 \in \mathbb{R}^{l \times n}, \mathbf{d}_{m,1} \in \mathbb{R}^l, \mathbf{d}_{M,1} \in \mathbb{R}^l\right\},
   
for **inequality constraints** in the form :math:`\mathbf{d}_{m,1} \leq \mathbf{C}_1\mathbf{x} \leq \mathbf{d}_{M,1}`;

.. math::
  
   \mathcal{C}_1 = \left\{ \mathbf{d}_{m,1} \in \mathbb{R}^n, \mathbf{d}_{M,1} \in \mathbb{R}^n\right\},
   
for **bounds** in the form :math:`\mathbf{d}_{m,1} \leq \mathbf{x} \leq \mathbf{d}_{M,1}`;

.. math::
  
   \mathcal{C}_1 = \left\{ \mathbf{C}_1 \in \mathbb{R}^{l \times n}, \mathbf{d}_1 \in \mathbb{R}^l\right\},

for **equality constraints** in the form :math:`\mathbf{C}_1\mathbf{x} = \mathbf{d}_1`.

A *Constraint* object is inherited from the base class ``OpenSoT::Constraint`` in `Constraint.h <https://advrhumanoids.github.io/OpenSoT/api/classOpenSoT_1_1Constraint.html#exhale-class-classopensot-1-1constraint>`__, where the method ``update(const Vector_type& x)`` must be implemented to assign:

- the matrix :math:`\mathbf{C}` as ``Eigen::MatrixXd _Aineq`` and the vectors :math:`\mathbf{d}_m` and :math:`\mathbf{d}_M` as ``Eigen::VectorXd _bLowerBound`` and ``Eigen::VectorXd _bUpperBound`` for an **inequality constraint**;
- the vectors :math:`\mathbf{d}_m` and :math:`\mathbf{d}_M` as ``Eigen::VectorXd _lowerBound`` and ``Eigen::VectorXd _upperBound`` for a **bound**;
- the matrix :math:`\mathbf{C}` as ``Eigen::MatrixXd _Aeq`` and the vector :math:`\mathbf{d}```Eigen::VectorXd _beq`` for an **equality constraint**. 

.. warning::
   Despite the ``Constraint.h`` calss support the definition of all the aforemenioned type of constraints, only the **inequality constraint** and **bound** are used by the front-end solvers. Most of the time, **equality constraint** are implemented using **inequality constraint** by posing :math:`\mathbf{d}_m = \mathbf{d}_M = \mathbf{d}`.
   
Upon invoking the ``og(XBot::MatLogger2::Ptr logger)`` method, the internal matrices ``_Aeq`` and ``_Aineq``, as well as the vectors ``_beq``, ``_bLowerbound``, ``_bUpperbound``, ``_lowerbound``, and ``_upperbound``  are stored in the file specified within the log. To record additional data, you must implement the virtual method ``_log(XBot::MatLogger2::Ptr logger)``.
    
SubConstraint
-------------
A *SubConstraint* comprises a specific number of rows from a *Constraint*. The *SubConstraint* class facilitates the selection of adjacent and non-adjacent rows from a constraint by extracting sub-matrices and sub-vectors.

.. note::
   *SubConstraints* **are always implemented as inequality constraints or equality constraints, NOT bounds!**

A *SubConstraints* can be instantiated from a *Constraint* using the ``SubConstraint(ConstraintPtr constrPtr, const std::list<unsigned int> rowIndices)`` constructor found in `SubConstraint.h <https://advrhumanoids.github.io/OpenSoT/api/classOpenSoT_1_1SubConstraint.html>`__.

.. note::
   A *SubConstraint* holds a reference to the *Constraint* that was used to instantiate it. As a result, any modifications applied to the primary *Constraint* are reflected in the *SubCOnstraint*, and vice versa. **When the** ``update(const Vector_type &x)`` **method is invoked on the** *SubConstraint*, **the referenced** *Constraint* **is also updated.**
      
A common application of a *SubConstraint* is to focus on a specific portion of a constraint. For instance, in a *Joint Limits Constraint*, a *SubCOnstraint* can be employed to do not consider limitations on the floating-base part of the state.

BilateralConstraint:
--------------------
A *constraint* can be created also from bare ``Eigen::MatrixXd`` and ``Eigen::VectorXd`` using the ``BilateralConstraint`` class in `BilateralConstraint.h <file:///home/enrico/catkin_ws/external/OpenSoT/docs/build/html/api/classOpenSoT_1_1constraints_1_1BilateralConstraint.html>`__:

.. code-block:: cpp
   
   //Creates a constraint
   Eigen::MatrixXd C(2,2); C.setIdentity();
   Eigen::VectorXd l(2), u(2);
   l[0] = l[1] = -10.;
   u[0] = u[1] =  10.;
   auto c1 = std::make_shared<OpenSoT::constraint::BilateralConstraint>("constraint1", C, l, u);

