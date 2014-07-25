% compare different QP solvers' performance on CUTEr test set problem

% Number of runs to average runtime
nruns = 3;

% dense or sparse
dense = 0;

%% get paths to third party QP solvers
% OOQP
addpath('~/extra/matlab/OOQP-0.99.22/src/Mex');
% QPOASES
addpath('..')
% CPLEX
addpath('~/extra/cplex122/matlab', '-END')

% print license
cplexoptimset('cplex');
fprintf('\n\n')

%% load problems
cuter_problems;

fprintf('%4s %10s %5s %5s %-13s %-13s %-13s %-13s %-13s %-13s %-13s %-13s %-13s\n',...
	'nr', 'name', 'm', 'n', 'qpoass0', 'qpoass1', ...
	'quadprog', 'OOQP', 'qpOASESf', 'qpOASESd', 'CPLEXP', 'CPLEXD', 'CPLEXB');

warning off all

for problemnr = 1:length(problem)

	% filter for problem size
	pname = problem{problemnr}{1};
	M = problem{problemnr}{2};
	N = problem{problemnr}{3};
	%if M >= 100 || N >= 100
	%if M > 1000 || N > 300
	%if M > 1001 || N > 250
	if M > 1001 || N > 1000
	%if M > 5000 || N > 1000
	%if M >= 5000 || N >= 5000
	%if M >= 10000 || N >= 10000
	%if M < 5000 || N < 1000 || M >= 20000 || N >= 5000
		continue
	end

	fprintf('%4d %-10s %5d %5d ', problemnr, pname, M, N);
	prob = [];
	for solvernr = 1:9
		if solvernr < 4
			fprintf('%13s ', '');
			continue
		end
		fname = sprintf('solver_comparison/problem_%04d_solver%02d', ...
			problemnr, solvernr);
		% check if problem is already being or has been solved with solver
		if exist([fname, '.running'], 'file') || ...
			exist([fname, '.mat'], 'file')
			fprintf('%13s ', '');
			continue
		else
			eval(sprintf('save %s.running problemnr solvernr -ASCII', fname));
		end

		% load problem
		if isempty(prob)
			try
				evalc('prob = load_cuter(pname, ''marosmeszaros'')');
			catch
				continue
			end
			[S, QP] = cuter_init_problem(prob);

			x0 = S.x;
			y0 = S.y;

			S.maxiter = 10000;
			S.printlevel = 0;

			clear data
			data.S = S;
			data.QP = QP;

			eval(sprintf('save solver_comparison/problem_%04d.mat data', ...
				problemnr));
		else
			S = data.S;
			QP = data.QP;
		end

		QP.fH = full(QP.H);
		QP.fAeq = full(QP.Aeq);
		QP.fAineq = full(QP.Aineq);
		QP.fC = full(QP.C);
		QP.fCineq = full(QP.Cineq);
		QP.spH = sparse(QP.H);
		QP.spAeq = sparse(QP.Aeq);
		QP.spAineq = sparse(QP.Aineq);
		QP.spC = sparse(QP.C);
		QP.spCineq = sparse(QP.Cineq);

		% run solver
		clear status solver

		switch solvernr
		case {1, 2}
			solver.name = sprintf('qpoass%1d', solvernr-1);
			cmd = 'S.nitref = solvernr-1; S = qpoass(S); status = S.errcode;';
		case 3
			if dense
				QP.H = QP.fH;
				QP.Aeq = QP.fAeq;
				QP.Aineq = QP.fAineq;
			else
				QP.H = QP.spH;
				QP.Aeq = QP.spAeq;
				QP.Aineq = QP.spAineq;
			end
			solver.name = 'quadprog';
			cmd = ['[x,fval,exitflag,output,lambda] = ' ...
				'quadprog(QP); status = exitflag - 1;'];
		case 4
			if dense
				QP.H = QP.fH;
				QP.Aeq = QP.fAeq;
				QP.Cineq = QP.fCineq;
			else
				QP.H = QP.spH;
				QP.Aeq = QP.spAeq;
				QP.Cineq = QP.spCineq;
			end
			solver.name = 'OOQP';
			cmd = ['[status, x, gamma_, phi_, y, z, lambda, pi] = ' ...
				'ooqp_dense(QP.f, QP.H, QP.lb, QP.ub, QP.Aeq, QP.beq, ' ...
				'QP.Cineq, QP.clineq, QP.cuineq, ''no'');'];
		case {5, 6}
			if dense
				QP.H = QP.fH;
				QP.C = QP.fC;
			else
				QP.H = QP.spH;
				QP.C = QP.spC;
			end
			switch solvernr
			case 5
				solver.name = 'qpOASESf';
				options = qpOASES_options('fast', 'maxIter', 10000, ...
					'initialStatusBounds', 1);
			case 6
				solver.name = 'qpOASESd';
				options = qpOASES_options('default', 'maxIter', 10000);
			end
			cmd = ['[x,obj,status,nWSRout,y] = qpOASES(QP.H, QP.f, ' ...
				'QP.C, QP.lb, QP.ub, QP.cl, QP.cu, [], options);'];
		case {7, 8, 9}
			if dense
				QP.H = QP.fH;
				QP.Aeq = QP.fAeq;
				QP.Aineq = QP.fAineq;
			else
				QP.H = QP.spH;
				QP.Aeq = QP.spAeq;
				QP.Aineq = QP.spAineq;
			end
			QP.options = cplexoptimset('cplex');
			QP.options.threads = 1;
			switch solvernr
			case 7 % primal
				solver.name = 'CPLEXP';
				QP.options.qpmethod = 1; 
			case 8 % dual 
				solver.name = 'CPLEXD';
				QP.options.qpmethod = 2; 
			case 9 % barrier 
				solver.name = 'CPLEXB';
				QP.options.qpmethod = 4; 
			end
			cmd = ['[x,fval,exitflag,output,lambda] = ' ...
				'cplexqp(QP); status = exitflag - 1;'];
		end
		x = x0; y = y0;
		try 
			solver.res = evalc(cmd);
			if status == 0
				if (solvernr < 7 || solvernr > 9)
					tic;
					for i = 1:nruns % try to even out calling and copy overhead
						S = data.S;
					end
					t1 = toc;
					for i = 1:nruns
						S = data.S;
						solver.res = evalc(cmd);
					end
					t = (toc - t1) / nruns;
				else
					solver.t_internal = [0 0 0];
					output.time = 0;
					tic
					for i = 1:nruns
						S = data.S;
						solver.res = evalc(cmd);
						[t2, t3, t4] = cplexlink122;
						solver.t_internal = solver.t_internal + [t2, t3, t4];
					end
					t = toc / nruns;
					solver.t_internal = solver.t_internal / nruns;
				end
				ts = sprintf('%7.3f', t);
				solver.t = t;
			else
				solver.t = 1e40;
				ts = 'FAILURE';
			end
		catch
			status = -1;
			ts = 'ERROR';
			solver.t = 1e40;
		end

		switch solvernr
		case {1, 2}
			solver.x = S.x;
			solver.y = S.y;
		case 3
			ybnds = zeros(size(QP.lb));
			yy = zeros(prob.m,1);
			if status == 0
				ybnds = lambda.upper - lambda.lower;
				yy(QP.equatn) = lambda.eqlin;
				yy(~QP.equatn) = reshape(lambda.ineqlin, ...
					sum(~QP.equatn),2) * [1; -1];
			end
			solver.x = x;
			solver.y = [ybnds(:); yy(:)];
		case 4
			ybnds = phi_ - gamma_;
			yy = zeros(prob.m,1);
			yy(QP.equatn) = -y;
			yy(~QP.equatn) = -z;
			solver.x = x;
			solver.y = [ybnds(:); yy(:)];
		case {5, 6}
			solver.x = x;
			solver.y = -y;
		case {7, 8, 9}
			ybnds = zeros(size(QP.lb));
			yy = zeros(prob.m,1);
			if status == 0
				ybnds = lambda.upper - lambda.lower;
				yy(QP.equatn) = lambda.eqlin;
				yy(~QP.equatn) = reshape(lambda.ineqlin, ...
					sum(~QP.equatn),2) * [1; -1];
			end
			solver.x = x;
			solver.y = [ybnds(:); -yy(:)];
		end

		% check error and print
		x = solver.x;
		y = solver.y;
		[stat, feas, cmpl] = qpresidual(S.B, S.b1, S.C, S.cl1, S.cu1, x, y);
		solver.stat = stat;
		solver.feas = feas;
		solver.cmpl = cmpl;
		fprintf('%7s %5.0e ', ts, stat + feas + cmpl)

		% save results and clean up
		eval(sprintf('save %s.mat solver', fname))
		eval(sprintf('delete %s.running', fname));
	end

	fprintf('\n')
end

fprintf('\n')

