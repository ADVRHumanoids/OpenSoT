Variables
=========
In OpenSoT, the matrix :math:`\mathbf{A}` and the vectors :math:`\mathbf{b}` and :math:`\mathbf{c}` completely define a task, and in the same way the matrix :math:`\mathbf{C}` and the vectors :math:`\mathbf{d}_m` and :math:`\mathbf{d}_M` completely define a constraint. While writing the optimization problem using these matrices, the *optimization variables* are *implicit* and depends on how these matrices and vectors are written, in other words, we do not write the variables of the problem but we infer them from its structure.

For example, in differential inverse kinematics, a Cartesian task is defined as:

.. math::
	
	\mathbf{J}\mathbf{\dot{q}} = \begin{bmatrix} \mathbf{v}\\ \boldsymbol{\omega} \end{bmatrix}
	
which results in :math:`\mathcal{T} = \left\{ \mathbf{A} =\mathbf{J}, \mathbf{W} = \mathbf{I}, \mathbf{b} =\begin{bmatrix} \mathbf{v}\\ \boldsymbol{\omega} \end{bmatrix}  \right\}`. It is important to notice that the optimization variable :math:`\mathbf{x}` consists *only* in the joint velocities :math:`\mathbf{x} = \mathbf{\dot{q}}`.

However, there are other formulations which may contains different types of variables. For example, in floating-base inverse dynamics, the variables considered are the joint acceleartions :math:`\mathbf{\ddot{q}}` and contact forces :math:`\mathbf{F}`. In these cases, the optimization variables contains both accelerations and forces :math:`\mathbf{x} = \begin{bmatrix} \mathbf{\ddot{q}} \\ \mathbf{F} \end{bmatrix}`. Therefore, if we consider implicitely the optimization variables, the matrices and vectors forming the tasks will have to take into account both types of optimization variables when defining the task. A Cartesian task in acceleration defined as:

.. math::
	
	\mathbf{J}\mathbf{\ddot{q}} = \begin{bmatrix} \mathbf{a}\\ \boldsymbol{\dot{\omega}} \end{bmatrix}-\mathbf{\dot{J}}\mathbf{\dot{q}}

will result in:

.. math::

	\mathcal{T} = \left\{ \mathbf{A} =\left[ \mathbf{J} \quad \mathbf{0} \right], \mathbf{W} = \mathbf{I}, \mathbf{b} =\begin{bmatrix} \mathbf{a}\\ \boldsymbol{\dot{\omega}} \end{bmatrix}-\mathbf{\dot{J}}\mathbf{\dot{q}}   \right\}

where the :math:`\mathbf{A}` matrix has a structure that takes into account the variables we are considering. From a developer point of view, this is not convenient because depending on the number of contact forces, the size of the task matrix will be different requiring more code to consider a variable which in fact is not used in the task! This also makes more complicate to reuse the task, for example in differential inverse kineamtics at the acceleration level where contact forces and dynamics are not present. Ideally, a developer would implement the task regardless of the other variables but considering only the joint accelerations.

To solve this issue, in OpenSoT we introduced the concepts of *explicit variables* through the ``OpenSoT::AffineHelper`` class  in `Affine.h <https://advrhumanoids.github.io/OpenSoT/api/file__home_runner_work_OpenSoT_OpenSoT_include_OpenSoT_utils_Affine.h.html#file-home-runner-work-opensot-opensot-include-opensot-utils-affine-h>`__. An *affine* variable is defined as:

.. math::

	\mathbf{y} = \mathbf{M}\mathbf{x} + \mathbf{q}
	
and can be used to define (or extract) explicit variables from :math:`\mathbf{x}`. 
For example, if :math:`\mathbf{x} = \begin{bmatrix} \mathbf{\ddot{q}} \\ \mathbf{F} \end{bmatrix}` with :math:`\mathbf{\ddot{q}} \in \mathbb{R}^n` and :math:`\mathbf{F} \in \mathbb{R}^{3c}`, then:

.. math::

	\mathbf{\ddot{q}} = \mathbf{M}_{\mathbf{\ddot{q}}}\mathbf{x}
and

.. math::

	\mathbf{F} = \mathbf{M}_{\mathbf{F}}\mathbf{x}
	
with :math:`\mathbf{M}_{\mathbf{\ddot{q}}} = \left[ \mathbf{I}_{n \times n} \quad \mathbf{0}_{n \times 3c} \right]` and :math:`\mathbf{M}_{\mathbf{F}} = \left[ \mathbf{0}_{3c \times n} \quad \mathbf{I}_{3c \times 3c} \right]`. In this way is possible to write task/constraints matrices and vectors that are independent from the definition of the optimization variables, in fact, if we consider again the Cartesian task in acceleration, we can write it as:

.. math::

	\mathcal{T} = \left\{ \mathbf{\bar{A}} =\mathbf{J}, \mathbf{W} = \mathbf{I}, \mathbf{b} =\begin{bmatrix} \mathbf{a}\\ \boldsymbol{\dot{\omega}} \end{bmatrix}-\mathbf{\dot{J}}\mathbf{\dot{q}}   \right\}
	
and compute the task matrix that will be used by the solver as: 

.. math::

	\mathbf{A} = \mathbf{\bar{A}}\mathbf{M}_{\mathbf{\ddot{q}}}
	
Variable usage
--------------
To create the vector of variables :math:`\mathbf{x}` we stack together all the variables we have in our optimization problem. This is done by the ``OpenSoT::OptvarHelper::VariableVector`` type. With the ``emplace_back(variable_name, variable_size)`` method we create and stack the variables. For example, for the previous inverse dynamics case, if :math:`n=19` and :math:`c=4`, we will have:

.. code-block::

	OpenSoT::OptvarHelper::VariableVector x;
	variable_name_dims.emplace_back("qddot", 19);
	variable_name_dims.emplace_back("F1", 3);
	variable_name_dims.emplace_back("F2", 3);
	variable_name_dims.emplace_back("F3", 3);
	variable_name_dims.emplace_back("F4", 3);
	
It is then possible to retrieve the variable by name: 

.. code-block::

	AffineHelper qddot = x->getVariable("qddot");
	
Some tasks in OpenSoT can be constructed using a variable, for example the ``OpenSoT::tasks::acceleration::Cartesian`` in `Cartesian.h <https://advrhumanoids.github.io/OpenSoT/api/classOpenSoT_1_1tasks_1_1acceleration_1_1Cartesian.html>`__.
Tasks that accepts variables normally does not directly assign the ``Eigen::MatrixXd _A`` and ``Eigen::MatrixXd _b``, but first uses internal variables to compute the task matrix and constraints with the size related to the variable in use and then computes the ``_A`` and ``_b`` using the API from the  ``AffineHelper`` class. 
For example, considering the Cartesian task in acceleration, let consider to have the Jacobian ``Eigen::MatrixXd J`` and bias vector ``Eigen::VectorXd Jdotqdot`` and the variable ``AffineHelper qddot``, we can define the Cartesian acceleration as:

.. code-block::

	AffineHelper xddot = J*qddot + Jdotqdot

The Cartesian acceleration task is defined as the minimization of the Cartesian acceleration and the reference, named ``Eigen::VectorXd xddot_ref``, we can define:

.. code-block::

	AffineHelper cartesian_task = xddot - xddot_ref

Now, we can retrieve the ``_A`` and ``_b`` from the ``cartesian_task``:

.. code-block::

	_A = cartesian_task.getM();
    	_b = -cartesian_task.getq();
    	
Affine Task and Affine Constraint
---------------------------------
OpenSoT provides utilities to transform tasks and constraints written without variables. This is done by the ``OpenSoT::AffineUtilsAffineTask`` and ``OpenSoT::AffineUtilsAffineConstraint`` classes in `AffineUtils.h <https://advrhumanoids.github.io/OpenSoT/api/namespace_OpenSoT__AffineUtils.html>`__. This permits to include in a stack also tasks and constraints that are (or were) not written using the ``AffineHelper`` variable API (e.g. the tasks and constraints at velocity level).
