Task
====

In OpenSoT, **Tasks** are used to form a cost function. Given a *Task* :math:`\mathcal{T}_1`, this is defined by its associated matrices and vectors:

.. math::
  
   \mathcal{T}_1 = \left\{ \mathbf{A}_1 \in \mathbb{R}^{m \times n}, \mathbf{W}_1 \in \mathbb{R}^{m \times m}, \mathbf{b}_1 \in \mathbb{R}^{m}, \mathbf{c}_1  \in \mathbb{R}^{n}  \right\}.
   
Multiple *Tasks* can be summed together to form complex cost fucntions, e.g. :math:`\mathcal{T}_3 = \mathcal{T}_1 + \mathcal{T}_2`:

.. math::
  
   \mathcal{T}_3 = \left\{ \begin{bmatrix}\mathbf{A}_1\newline \mathbf{A}_2 \end{bmatrix}, \begin{bmatrix}\mathbf{W}_1 & \mathbf{0}\newline \mathbf{0}   & \mathbf{W}_2 \end{bmatrix}, \begin{bmatrix} \mathbf{b}_1\newline \mathbf{b}_2 \end{bmatrix}, \mathbf{c}_1 + \mathbf{c}_2  \right\}.

The scalar cost function associated to :math:`\mathcal{T}_3` will be:

.. math::

   \begin{align}
   \mathcal{F} 
   %& = \lVert \begin{bmatrix}\mathbf{A}_1\newline \mathbf{A}_2 \end{bmatrix}\mathbf{x} -  \begin{bmatrix} \mathbf{b}_1\newline \mathbf{b}_2 \end{bmatrix}\rVert_{\begin{bmatrix}\mathbf{W}_1 & \mathbf{0}\newline \mathbf{0}   & \mathbf{W}_2 \end{bmatrix}} + \left(\mathbf{c}_1 + \mathbf{c}_2\right)^T\mathbf{x} = \newline
   & = \left( \begin{bmatrix}\mathbf{A}_1\newline \mathbf{A}_2 \end{bmatrix}\mathbf{x} -  \begin{bmatrix} \mathbf{b}_1\newline \mathbf{b}_2 \end{bmatrix} \right)^T \begin{bmatrix}\mathbf{W}_1 & \mathbf{0}\newline \mathbf{0}   & \mathbf{W}_2 \end{bmatrix} \left( \begin{bmatrix}\mathbf{A}_1\newline \mathbf{A}_2 \end{bmatrix}\mathbf{x} -  \begin{bmatrix} \mathbf{b}_1\newline \mathbf{b}_2 \end{bmatrix} \right) + \left(\mathbf{c}_1 + \mathbf{c}_2\right)^T\mathbf{x} = \newline
   %& = \left(\mathbf{x}^T \begin{bmatrix} \mathbf{A}_1^T & \mathbf{A}_2^T \end{bmatrix} - \begin{bmatrix}\mathbf{b}_1^T & \mathbf{b}_2^T \end{bmatrix}\right) \begin{bmatrix}\mathbf{W}_1 & \mathbf{0}\newline \mathbf{0}   & \mathbf{W}_2 \end{bmatrix} \left( \begin{bmatrix}\mathbf{A}_1\newline \mathbf{A}_2 \end{bmatrix}\mathbf{x} -  \begin{bmatrix} \mathbf{b}_1\newline \mathbf{b}_2 \end{bmatrix} \right) + \left(\mathbf{c}_1 + \mathbf{c}_2\right)^T\mathbf{x} = \newline
   %& = \left( \begin{bmatrix}\mathbf{x}^T\mathbf{A}_1^T\mathbf{W}_1 & \mathbf{x}^T\mathbf{A}_2^T\mathbf{W}_2 \end{bmatrix} - \begin{bmatrix} \mathbf{b}_1^T\mathbf{W}_1 & \mathbf{b}_2^T\mathbf{W}_2 \end{bmatrix}\right) \left( \begin{bmatrix}\mathbf{A}_1\newline \mathbf{A}_2 \end{bmatrix}\mathbf{x} -  \begin{bmatrix} \mathbf{b}_1\newline \mathbf{b}_2 \end{bmatrix} \right) + \left(\mathbf{c}_1 + \mathbf{c}_2\right)^T\mathbf{x} = \newline
   %& = \mathbf{x}^T\mathbf{A}_1^T\mathbf{W}_1\mathbf{A}_1\mathbf{x} + \mathbf{x}^T\mathbf{A}_2^T\mathbf{W}_2\mathbf{A}_2\mathbf{x} -2\mathbf{b}_1^T\mathbf{W}_1\mathbf{A}_1\mathbf{x} -2\mathbf{b}_2^T\mathbf{W}_2\mathbf{A}_2\mathbf{x} + \left(\mathbf{c}_1 + \mathbf{c}_2\right)^T\mathbf{x} = \newline
   & = \sum_{i = 1}^{2} \lVert \mathbf{A}_i\mathbf{x} - \mathbf{b}_i\rVert_{\mathbf{W}_i} + \sum_{i = 1}^{2}\mathbf{c}_i^T\mathbf{x},
   \end{align}

i.e. a weighted sum of cost functions associated to each *Task*.

.. note:: 
   Most of the time, the :math:`\mathbf{c}` term is used to implement the *Lasso* or an *L-1 norm*; thus, it is typically not utilized.
