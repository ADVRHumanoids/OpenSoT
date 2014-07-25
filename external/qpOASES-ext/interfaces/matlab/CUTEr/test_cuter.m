pname = 'KSIP';
fprintf('Loading problem %s...\n', pname);
prob = load_cuter(pname, 'marosmeszaros');
[S, QP] = cuter_init_problem(prob);

addpath ..

fprintf(' n = %d\n m = %d\n', prob.n, prob.m)

nWSRout = 0;
options = qpOASES_options('default', 'maxIter', -1, 'printLevel', -1);
tic
[x, fval, status, nWSRout, y] = qpOASES(full(QP.H), QP.f, ...
	full(QP.C), QP.lb, QP.ub, QP.cl, QP.cu, [], options);
t = toc;
% check error and print
[stat, feas, cmpl] = qpresidual(S.B, S.b1, S.C, S.cl1, S.cu1, x, -y);
fprintf('%d iters in %.3fs to tolerance %.2e\n', nWSRout, t, stat + feas + cmpl)
fprintf('Status: %d\n', status)

