Intro
===================================

OpenSoT provides an API to setup and solve generic Least-Squares like optimization problems in the form:

.. math:: 
   
   \begin{align}
   &\min\limits_{\mathbf{x}} \lVert \mathbf{A}\mathbf{x} - \mathbf{b}\rVert_{\mathbf{W}} + \mathbf{c}^T\mathbf{x}+\epsilon\lVert \mathbf{x} \rVert \newline
   &s.t. \quad  \mathbf{d}_m\leq \mathbf{C}\mathbf{x}\leq\mathbf{d}_M 
   \end{align} 
   
   
with :math:`\mathbf{x} \in \mathbb{R}^n`, :math:`\mathbf{A} \in \mathbb{R}^{m \times n}`, :math:`\mathbf{b} \in \mathbb{R}^{m}`, :math:`\mathbf{W} \in \mathbb{R}^{m \times m}`, :math:`\mathbf{c} \in \mathbb{R}^{n}`, :math:`\mathbf{C} \in \mathbb{R}^{l \times n}`, :math:`\mathbf{d}_m` and  :math:`\mathbf{d}_M \in \mathbb{R}^{l}`, and :math:`\epsilon \in \mathbb{R}^+`.  This optimization problem is equivalent to the following QP problem:

.. math::

   \begin{align}
   &\min\limits_{\mathbf{x}} \mathbf{x}^T\mathbf{H}\mathbf{x} + \mathbf{g}^T\mathbf{x} \newline
   &s.t. \quad \mathbf{d}_m\leq \mathbf{C}\mathbf{x}\leq\mathbf{d}_M
   \end{align}

with :math:`\mathbf{H} = \mathbf{A}^T\mathbf{W}\mathbf{A} + \epsilon\mathbf{I}` and :math:`\mathbf{g}^T = \mathbf{c}^T - 2\mathbf{b}^T\mathbf{W}\mathbf{A}`, and can be solved by dedicated solvers.

In OpenSoT, **Tasks** are used to form a cost function. Given a *Task* :math:`\mathcal{T}_1`, this is defined by its associated matrices and vectors:

.. math::
  
   \mathcal{T}_1 = \left\{ \mathbf{A}_1, \mathbf{W}_1, \mathbf{b}_1, \mathbf{c}_1  \right\}.
   
Multiple *Tasks* can be summed together to form complex cost fucntions, e.g. :math:`\mathcal{T}_3 = \mathcal{T}_1 + \mathcal{T}_2`:

.. math::
  
   \mathcal{T}_3 = \left\{ \begin{bmatrix}\mathbf{A}_1\newline \mathbf{A}_2 \end{bmatrix}, \begin{bmatrix}\mathbf{W}_1 & \mathbf{0}\newline \mathbf{0}   & \mathbf{W}_2 \end{bmatrix}, \begin{bmatrix} \mathbf{b}_1\newline \mathbf{b}_2 \end{bmatrix}, \mathbf{c}_1 + \mathbf{c}_2  \right\}.

The scalar cost function associated to :math:`\mathcal{T}_3` will be:

.. math::

   \begin{align}
   \mathcal{F} & = \lVert \begin{bmatrix}\mathbf{A}_1\newline \mathbf{A}_2 \end{bmatrix}\mathbf{x} -  \begin{bmatrix} \mathbf{b}_1\newline \mathbf{b}_2 \end{bmatrix}\rVert_{\begin{bmatrix}\mathbf{W}_1 & \mathbf{0}\newline \mathbf{0}   & \mathbf{W}_2 \end{bmatrix}} + \left(\mathbf{c}_1 + \mathbf{c}_2\right)^T\mathbf{x} = \newline
   & = \left( \begin{bmatrix}\mathbf{A}_1\newline \mathbf{A}_2 \end{bmatrix}\mathbf{x} -  \begin{bmatrix} \mathbf{b}_1\newline \mathbf{b}_2 \end{bmatrix} \right)^T \begin{bmatrix}\mathbf{W}_1 & \mathbf{0}\newline \mathbf{0}   & \mathbf{W}_2 \end{bmatrix} \left( \begin{bmatrix}\mathbf{A}_1\newline \mathbf{A}_2 \end{bmatrix}\mathbf{x} -  \begin{bmatrix} \mathbf{b}_1\newline \mathbf{b}_2 \end{bmatrix} \right) + \left(\mathbf{c}_1 + \mathbf{c}_2\right)^T\mathbf{x} = \newline
   & = \left(\mathbf{x}^T \begin{bmatrix} \mathbf{A}_1^T & \mathbf{A}_2^T \end{bmatrix} - \begin{bmatrix}\mathbf{b}_1^T & \mathbf{b}_2^T \end{bmatrix}\right) \begin{bmatrix}\mathbf{W}_1 & \mathbf{0}\newline \mathbf{0}   & \mathbf{W}_2 \end{bmatrix} \left( \begin{bmatrix}\mathbf{A}_1\newline \mathbf{A}_2 \end{bmatrix}\mathbf{x} -  \begin{bmatrix} \mathbf{b}_1\newline \mathbf{b}_2 \end{bmatrix} \right) + \left(\mathbf{c}_1 + \mathbf{c}_2\right)^T\mathbf{x} = \newline
   & = \left( \begin{bmatrix}\mathbf{x}^T\mathbf{A}_1^T\mathbf{W}_1 & \mathbf{x}^T\mathbf{A}_2^T\mathbf{W}_2 \end{bmatrix} - \begin{bmatrix} \mathbf{b}_1^T\mathbf{W}_1 & \mathbf{b}_2^T\mathbf{W}_2 \end{bmatrix}\right) \left( \begin{bmatrix}\mathbf{A}_1\newline \mathbf{A}_2 \end{bmatrix}\mathbf{x} -  \begin{bmatrix} \mathbf{b}_1\newline \mathbf{b}_2 \end{bmatrix} \right) + \left(\mathbf{c}_1 + \mathbf{c}_2\right)^T\mathbf{x} = \newline
   & = \mathbf{x}^T\mathbf{A}_1^T\mathbf{W}_1\mathbf{A}_1\mathbf{x} + \mathbf{x}^T\mathbf{A}_2^T\mathbf{W}_2\mathbf{A}_2\mathbf{x} -2\mathbf{b}_1^T\mathbf{W}_1\mathbf{A}_1\mathbf{x} -2\mathbf{b}_2^T\mathbf{W}_2\mathbf{A}_2\mathbf{x} + \left(\mathbf{c}_1 + \mathbf{c}_2\right)^T\mathbf{x} = \newline
   & = \sum_{i = 1}^{2} \lVert \mathbf{A}_i\mathbf{x} - \mathbf{b}_i\rVert_{\mathbf{W}_i} + \sum_{i = 1}^{2}\mathbf{c}_i^T\mathbf{x},
   \end{align}

i.e. a weighted sum of cost functions associated to each *Task*.

.. note:: 
   Most of the time, the :math:`\mathbf{c}` term is used to implement the *Lasso* or an *L-1 norm*; thus, it is typically not utilized.