param = 'N=1000';
pname = {
	'NCVXQP1',
	'NCVXQP2',
	'NCVXQP3',
	'NCVXQP4',
	'NCVXQP5',
	'NCVXQP6',
	'NCVXQP7',
	'NCVXQP8',
	'NCVXQP9'
};

addpath ..

for i = 1:length(pname)
	delete mcuter.mex*
	prob = load_cuter(pname{i}, 'mastsif', param);

	[S, QP] = cuter_init_problem(prob);
	%fprintf(' n = %d\n m = %d\n', prob.n, prob.m)

	nWSRout = 0;
	options = qpOASES_options('default', 'maxIter', 100000, ...
		'printLevel', 0, 'enableEqualities', 1);
	tic
	[x, fval, status, nWSRout, y] = qpOASES(sparse(QP.H), QP.f, ...
		sparse(QP.C), QP.lb, QP.ub, QP.cl, QP.cu, [], options);
	t = toc;
	% check error and print
	[stat, feas, cmpl] = qpresidual(S.B, S.b1, S.C, S.cl1, S.cu1, x, -y);
	rhores = max([stat, feas, cmpl]);

	n = prob.n;
	m = prob.m;
	tol = 1.0e-8;

	% standard form A x >= b
	H = S.B;
	g = S.b1;
	idxE = abs(S.cu1 - S.cl1) <= tol;
	nEqs = sum(idxE);
	idxI = ~idxE;
	A = [S.C(idxE,:); S.C(idxI,:); -S.C(idxI,:)];
	b = [S.cl1(idxE); S.cl1(idxI); -S.cu1(idxI)];
	yy = [y(idxE); max(0,y(idxI)); max(0,-y(idxI))];

	idxE = (1:size(A,1))' <= nEqs;
	idxI = (1:size(A,1))' > nEqs;

	idxAplus = idxE | (A * x - b <= tol & yy > tol);
	%idxWeak = idxI & (A * x - b <= tol & yy <= tol);

	% compute nullspace of equality and strongly active constraints
	Z = null(A(idxAplus,:));

	% add feasible cone directions for weakly active constraints
%	addweak = zeros(1,0);
%	for j = find(idxWeak)
%		% check feasibility for equalities and strongly active constraints
%		if norm(A(idxAplus,:) * A(j,:)',inf) > tol
%			continue
%		end
%		% check feasibility for weakly active constraints
%		if all(A(addweak,:) * A(j,:)' >= -tol)
%			addweak = [addweak j];
%		end
%	end
%	ZZ = [Z, A(addweak,:)'];

	Hred = Z'*H*Z;

	mineig=min([inf, min(eig(Hred))]);

	fprintf('%10s & %4d & %4d & %4d & %9.2e & %9.2e\n', ...
		pname{i}, n, m, nWSRout, mineig, rhores)
end

