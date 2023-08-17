Stack
=====

The concept of *Stack of Tasks* was first introduced by *Nicolas Mansard et al.* in the seminal work: *A Versatile Generalized Inverted Kinematics Implementation for Collaborative Working Humanoid Robots: The Stack of Tasks* (`paper <https://homepages.laas.fr/ostasse/mansard_icar09.pdf>`__). In OpenSoT a **stack** consists in one or more tasks and constraints and their relations.  

A single *task* can be used to initialize a *stack* by using the ``/=`` operator, e.g.:

.. code-block:: cpp
   
   //Creates a task
   Eigen::MatrixXd A(2,2); A.rand();
   Eigen::VectorXd b(2); b.rand();
   auto t1 = std::make_shared<OpenSoT::tasks::GenericTask>("task1", A, b);
	
   //Creates a stack
   OpenSoT::AutoStack::Ptr stack /= t1; 


Multiple *Tasks* can be summed together to form complex cost functions, for instance :math:`\mathcal{T}_3 = \mathcal{T}_1 + \mathcal{T}_2`:

.. math::
  
   \mathcal{T}_3 = \left\{ \begin{bmatrix}\mathbf{A}_1\newline \mathbf{A}_2 \end{bmatrix}, \begin{bmatrix}\mathbf{W}_1 & \mathbf{0}\newline \mathbf{0}   & \mathbf{W}_2 \end{bmatrix}, \begin{bmatrix} \mathbf{b}_1\newline \mathbf{b}_2 \end{bmatrix}, \mathbf{c}_1 + \mathbf{c}_2  \right\}.

The scalar cost function associated to :math:`\mathcal{T}_3` will be:

.. math::

   \begin{align}
   \mathcal{F}_3 
   %& = \lVert \begin{bmatrix}\mathbf{A}_1\newline \mathbf{A}_2 \end{bmatrix}\mathbf{x} -  \begin{bmatrix} \mathbf{b}_1\newline \mathbf{b}_2 \end{bmatrix}\rVert_{\begin{bmatrix}\mathbf{W}_1 & \mathbf{0}\newline \mathbf{0}   & \mathbf{W}_2 \end{bmatrix}} + \left(\mathbf{c}_1 + \mathbf{c}_2\right)^T\mathbf{x} = \newline
   & = \left( \begin{bmatrix}\mathbf{A}_1\newline \mathbf{A}_2 \end{bmatrix}\mathbf{x} -  \begin{bmatrix} \mathbf{b}_1\newline \mathbf{b}_2 \end{bmatrix} \right)^T \begin{bmatrix}\mathbf{W}_1 & \mathbf{0}\newline \mathbf{0}   & \mathbf{W}_2 \end{bmatrix} \left( \begin{bmatrix}\mathbf{A}_1\newline \mathbf{A}_2 \end{bmatrix}\mathbf{x} -  \begin{bmatrix} \mathbf{b}_1\newline \mathbf{b}_2 \end{bmatrix} \right) + \left(\mathbf{c}_1 + \mathbf{c}_2\right)^T\mathbf{x} = \newline
   %& = \left(\mathbf{x}^T \begin{bmatrix} \mathbf{A}_1^T & \mathbf{A}_2^T \end{bmatrix} - \begin{bmatrix}\mathbf{b}_1^T & \mathbf{b}_2^T \end{bmatrix}\right) \begin{bmatrix}\mathbf{W}_1 & \mathbf{0}\newline \mathbf{0}   & \mathbf{W}_2 \end{bmatrix} \left( \begin{bmatrix}\mathbf{A}_1\newline \mathbf{A}_2 \end{bmatrix}\mathbf{x} -  \begin{bmatrix} \mathbf{b}_1\newline \mathbf{b}_2 \end{bmatrix} \right) + \left(\mathbf{c}_1 + \mathbf{c}_2\right)^T\mathbf{x} = \newline
   %& = \left( \begin{bmatrix}\mathbf{x}^T\mathbf{A}_1^T\mathbf{W}_1 & \mathbf{x}^T\mathbf{A}_2^T\mathbf{W}_2 \end{bmatrix} - \begin{bmatrix} \mathbf{b}_1^T\mathbf{W}_1 & \mathbf{b}_2^T\mathbf{W}_2 \end{bmatrix}\right) \left( \begin{bmatrix}\mathbf{A}_1\newline \mathbf{A}_2 \end{bmatrix}\mathbf{x} -  \begin{bmatrix} \mathbf{b}_1\newline \mathbf{b}_2 \end{bmatrix} \right) + \left(\mathbf{c}_1 + \mathbf{c}_2\right)^T\mathbf{x} = \newline
   %& = \mathbf{x}^T\mathbf{A}_1^T\mathbf{W}_1\mathbf{A}_1\mathbf{x} + \mathbf{x}^T\mathbf{A}_2^T\mathbf{W}_2\mathbf{A}_2\mathbf{x} -2\mathbf{b}_1^T\mathbf{W}_1\mathbf{A}_1\mathbf{x} -2\mathbf{b}_2^T\mathbf{W}_2\mathbf{A}_2\mathbf{x} + \left(\mathbf{c}_1 + \mathbf{c}_2\right)^T\mathbf{x} = \newline
   & = \sum_{i = 1}^{2} \lVert \mathbf{A}_i\mathbf{x} - \mathbf{b}_i\rVert_{\mathbf{W}_i} + \sum_{i = 1}^{2}\mathbf{c}_i^T\mathbf{x},
   \end{align}

i.e. a weighted sum of cost functions associated to each *Task*.

Multiple *tasks* can be summed up using the ``+`` operator. Relative weights can be set by using the ``*`` operator, e.g.:

.. code-block:: cpp

   //Creates tasks
   Eigen::MatrixXd A(2,2);
   Eigen::VectorXd b(2);
   auto t1 = std::make_shared<OpenSoT::tasks::GenericTask>("task1", A.rand(), b.rand());
   auto t2 = std::make_shared<OpenSoT::tasks::GenericTask>("task2", A.rand(), b.rand());
	
   //Creates a stack
   OpenSoT::AutoStack::Ptr stack = t1 + 0.1*t2; 

relative weights can be either scalar or matrices of the right size. 

A *subtask* can be used as well inside a *stack*. A simple way to create a *subtask* is to use the ``%`` operator. Notice that is not important to keep track of the generated *subtask* once it is inserted inside a *stack*, e.g.: 

.. code-block:: cpp

   //Creates tasks
   Eigen::MatrixXd A(3,3);
   Eigen::VectorXd b(3);
   auto t1 = std::make_shared<OpenSoT::tasks::GenericTask>("task1", A.rand(), b.rand());
   auto t2 = std::make_shared<OpenSoT::tasks::GenericTask>("task2", A.rand(), b.rand());
	
   //Creates a stack
   std::list<unsigned int> idx {0, 1};
   OpenSoT::AutoStack::Ptr stack = t1%idx + 0.1*t2; 

A *constraint* can be associated to *task* or a *stack* using the ``<<`` operator:

.. code-block:: cpp

   //Creates tasks
   Eigen::MatrixXd A(2,2);
   Eigen::VectorXd b(2);
   auto t1 = std::make_shared<OpenSoT::tasks::GenericTask>("task1", A.rand(), b.rand());
   auto t2 = std::make_shared<OpenSoT::tasks::GenericTask>("task2", A.rand(), b.rand());
   
   //Creates a constraint
   Eigen::MatrixXd C(2,2); C.setIdentity();
   Eigen::VectorXd l(2), u(2);
   l[0] = l[1] = -10.;
   u[0] = u[1] =  10.;
   auto c1 = std::make_shared<OpenSoT::constraint::BilateralConstraint>("constraint1", C, l, u);
	
   //Creates a stack
   OpenSoT::AutoStack::Ptr stack = (t1 + 0.1*t2)<<c1; 

Multiple *constraints* can be included using ``<<``. *Tasks* can be also used as *equality constraints* and directly added:

.. code-block:: cpp

   //Creates tasks
   Eigen::MatrixXd A(2,2);
   Eigen::VectorXd b(2);
   auto t1 = std::make_shared<OpenSoT::tasks::GenericTask>("task1", A.rand(), b.rand());
   auto t2 = std::make_shared<OpenSoT::tasks::GenericTask>("task2", A.rand(), b.rand());
   
   //Creates a constraint
   Eigen::MatrixXd C(2,2); C.setIdentity();
   Eigen::VectorXd l(2), u(2);
   l[0] = l[1] = -10.;
   u[0] = u[1] =  10.;
   auto c1 = std::make_shared<OpenSoT::constraint::BilateralConstraint>("constraint1", C, l, u);
	
   //Creates a stack
   OpenSoT::AutoStack::Ptr stack /= t2
   stack<<t1<<c1; 
   
.. note::
   *Constraints* can also be directly added to *tasks*. However this feature is not commonly used preferring to associate the *constraint* directly to the *stack*.

The ``%`` operator can be used as well to *constraints* to create *subconstraints*.

All these operators consists in a *Domain Specific Language* (DSL) named *Math of Tasks* (`MoT <https://www.worldscientific.com/doi/abs/10.1142/S0219843621500080>`__) that permits to create complex QP problems.

The *stack* object is implemented through the ``OpenSoT::AutoStack`` class in `Autostack.h <https://advrhumanoids.github.io/OpenSoT/api/classOpenSoT_1_1AutoStack.html>`__. **An autostack carries pointers to the associated tasks and constraints, therefore when the autostack is updated through the** ``update(const Eigen::VectorXd & state)`` **method, also the internal tasks and constraints are updated.** 
