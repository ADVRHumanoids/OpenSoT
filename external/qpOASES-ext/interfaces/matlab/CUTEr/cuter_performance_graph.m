% create performance graph for solver comparison on CUTEr test set

cuter_problems;

solvers = 3:9;
nslv = size(solvers,2);

dense = 0;
OPT_THRES = 1e-4;
USE_INTERNAL_TIMING = 1;
bw = 1;

if dense
	tfactormax = 1e3;
else
	tfactormax = 5e3;
end
tfactor = linspace(0, tfactormax, 100*tfactormax+1);
performance = zeros(nslv, 100*tfactormax+1);

fprintf('%4s %10s %5s %5s %-13s %-13s %-13s %-13s %-13s %-13s %-13s\n',...
	'nr', 'name', 'm', 'n', ...
	'quadprog', 'OOQP', 'qpOASESf', 'qpOASESd', 'CPLEXP', 'CPLEXD', 'CPLEXB');

nproblems = 0;
for i = 1:length(problem)
	% load data from disk
	clear data
	try
		eval(sprintf('load solver_comparison/problem_%04d.mat', i));
	catch
		continue
	end

	if dense && size(data.S.C,2) > 250
		continue
	end

	nproblems = nproblems + 1;

	ts = cell(nslv,1);
	opt = 1e40 * ones(nslv,1);
	t = 1e40 * ones(nslv,1);
	for j = 1:nslv
		k = solvers(j);

		fname = sprintf('solver_comparison/problem_%04d_solver%02d', i, k);
		try
			eval(sprintf('load %s.mat', fname));
		catch
			ts{j} = 'FAILURE';
			continue
		end

		% check for optimality
		opt(j) = solver.stat + solver.feas + solver.cmpl;
		if USE_INTERNAL_TIMING && isfield(solver, 't_internal') ...
			&& norm(solver.t_internal) > 0 && k >= 7 && solver.t < 1e40
			% all cplexlink122
			%t(j) = solver.t_internal(1);

			% cplexlink122 without constructors
			%t(j) = solver.t_internal(2);

			% only cplexlink122 solve
			t(j) = solver.t_internal(3);
		else
			t(j) = solver.t;
		end
		if t(j) >= 1e40
			ts{j} = 'FAILURE';
		elseif ~(opt(j) < OPT_THRES)
			t(j) = 1e40;
			ts{j} = 'TOO BAD';
		else
			ts{j} = sprintf('%7.3f', t(j));
		end
	end

	pname = problem{i}{1};
	M = problem{i}{2};
	N = problem{i}{3};
	fprintf('%4d %-10s %5d %5d ', i, pname, M, N);
	for j = 1:nslv
		fprintf('%7s %5.0e ', ts{j}, opt(j));
	end
	fprintf('\n');

	tmin = min(t);
	if tmin >= 1e40 % no solver found solution
		continue
	end

	for j = 1:nslv
		performance(j,:) = performance(j,:) + (t(j) <= tmin * tfactor);
	end
end

% normalize
performance = 100 * performance / nproblems;

% plot
if bw
	%linespec = {'-k.', '-kx', '-kv', '-ko', '-k*', '-ks', '-kd', '-k^'};
	linespec = {'-k', '-k', '--k', '-k', '--k', '-k', ':k', '-k^'};
	linewidth = [0.5 0.5 2 2 1 1 0.5 0.5];
else
	linespec = {'-b', '-g', '-r', '-c', '-m', '-y', '-k', '-b'};
	linewidth = [1 1 1 1 1 1 1 1];
end
for j = 2:nslv
	steps = diff(performance(j,:));
	steps = [1 steps] | [steps 1];
	semilogx(tfactor(steps), performance(j,steps), linespec{j}, ...
		'MarkerSize', 3, 'LineWidth', linewidth(j))
	hold on
end
hold off
%legend('quadprog', 'OOQP', 'qpOASESmpc', 'qpOASES', ...
legend('OOQP', 'qpOASESmpc', 'qpOASES', ...
	'CPLEXP', 'CPLEXD', 'CPLEXB', 'Location', 'SE')
xlabel('Time factor')
ylabel('Percentage of problems solved')
%grid on
set(gca, 'MinorGridLineStyle', 'none')
axis([ 1 tfactormax 0 100 ])

