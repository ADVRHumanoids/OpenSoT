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

