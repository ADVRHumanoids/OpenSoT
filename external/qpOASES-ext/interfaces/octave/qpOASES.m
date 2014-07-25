%qpOASES -- An Implementation of the Online Active Set Strategy.
%Copyright (C) 2007-2014 by Hans Joachim Ferreau, Andreas Potschka,
%Christian Kirches et al. All rights reserved.
%
%qpOASES is distributed under the terms of the
%GNU Lesser General Public License 2.1 in the hope that it will be
%useful, but WITHOUT ANY WARRANTY; without even the implied warranty
%of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
%See the GNU Lesser General Public License for more details.
%
%---------------------------------------------------------------------------------
%
%qpOASES solves (a series) of quadratic programming (QP) problems of the
%following form:
%
%                min   1/2*x'Hx + x'g
%                s.t.  lb  <=  x <= ub
%                      lbA <= Ax <= ubA  {optional}
%
%Call
%
%    [x,fval,exitflag,iter,lambda,workingSet] = 
%                     qpOASES( H,g,A,lb,ub,lbA,ubA{,options{,x0{,workingSet}}} )
%
%for solving the above-mentioned QP. H must be a symmetric (but possibly 
%indefinite) matrix and all vectors g, lb, ub, lbA, ubA have to be given
%as column vectors. Options can be generated using the qpOASES_options command, 
%otherwise default values are used. Optionally, an initial guess x0 or an
%initial guess for the working set can be specified. If no initial guesses are 
%provided, iterations start from the origin. 
%
%Call
%
%    [x,fval,exitflag,iter,lambda,workingSet] =
%                     qpOASES( H,g,lb,ub{,options{,x0{,workingSet}}} )
%
%for solving the above-mentioned QP without general constraints.
%
%
%Optional Outputs (only obj is mandatory):
%    x          -  optimal primal solution vector   (if status==0)
%    fval       -  optimal objective function value (if status==0)
%    exitflag   -   0: QP problem solved
%                   1: QP could not be solved within given number of iterations
%                  -1: QP could not be solved due to an internal error
%                  -2: QP is infeasible (and thus could not be solved)
%                  -3: QP is unbounded (and thus could not be solved)
%    iter       -  number of active set iterations actually performed
%    lambda     -  optimal dual solution vector (if status==0)
%    workingSet -  the working set at point x. The working set is a subset
%                  of the active set (which is the set of all indices
%                  corresponding to bounds/constraints that hold with 
%                  equality). The working set corresponds to bound/
%                  constraint row vectors forming alinear independent set.
%                  The first nV entries correspond to the bounds, the last
%                  nC to the constraints.
%                  The working set is encoded as follows:
%                   1: bound/constraint at its upper bound
%                   0: bound/constraint not at any bound
%                  -1: bound/constraint at its lower bound
%
%
%If not a single QP but a sequence of QPs with varying vectors is to be solved,
%the i-th QP is given by the i-th columns of the QP vectors g, lb, ub, lbA, ubA
%(i.e. they are matrices in this case). Both matrices H and A remain constant.
%
%See also QPOASES_OPTIONS, QPOASES_SEQUENCE
%
%
%For additional information see the qpOASES User's Manual or
%visit http://www.qpOASES.org/.
%
%Please send remarks and questions to support@qpOASES.org!
