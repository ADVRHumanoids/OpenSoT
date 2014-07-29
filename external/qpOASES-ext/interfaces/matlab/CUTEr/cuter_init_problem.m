function [S, QP] = cuter_init_problem(prob)
% Initialize qpoass structure S from CUTEr problem structure prob. Optionally
% also initialize quadprog structure QP.
%

x0 = zeros(size(prob.x));
try
	[coff,C] = cuter_cons(x0);
catch
	coff=[];
	C=zeros(0,size(x0,1));
end
[f,b1] = cuter_obj(x0);
if (length(prob.v)>0)
	S.B = cuter_hess(prob.x, prob.v);
else
	S.B = cuter_hess(prob.x);
end
cl = prob.cl - coff;
cu = prob.cu - coff;
cu(prob.equatn) = cl(prob.equatn); % adjust equations
QP.equatn = (cu - cl) <= 1e-14;
S.C = [eye(prob.n); C];
S.cl1 = [prob.bl; cl];
S.cu1 = [prob.bu; cu];
S.b1 = b1;
S.asi = zeros(prob.m + prob.n, 1);
S.cl0 = zeros(size(S.cu1));
S.cu0 = zeros(size(S.cu1));
S.cl0(1:prob.n) = 0;
S.asi(1:prob.n) = -1;
S.cu0 = zeros(size(S.cu1));
% set primal variables
S.x = zeros(size(prob.x));
S.y = zeros(size(S.cl1));
S.Cx = S.C * S.x;

if nargout >= 2
	QP.H = S.B;
	QP.f = S.b1;
	QP.C = C;
	Ceq = C(QP.equatn,:);
	QP.Cineq = C(~QP.equatn,:);
	QP.Aeq = Ceq;
	QP.Aineq = [QP.Cineq; -QP.Cineq];
	QP.beq = cl(QP.equatn);
	QP.cu = cu; 
	QP.cl = cl;
	QP.cuineq = cu(~QP.equatn); 
	QP.clineq = cl(~QP.equatn);
	QP.bineq = [QP.cuineq; -QP.clineq];
	QP.lb = prob.bl;
	QP.ub = prob.bu;
	QP.options = optimset('MaxIter', 10000, 'LargeScale', 'off', ...
		'Display', 'Off');
	QP.solver = 'quadprog';
end

