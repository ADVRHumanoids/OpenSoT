function prob = load_cuter(problem_name, testset, param)
% prob = load_cuter(problem_name, testset)
% Load problem problem_name from CUTEr test set testset. If the problem has
% already been compiled, just call cuter_setup, if not, compile first.
%

cuter_dir = getenv('CUTER');
cuter_bin_dir = getenv('MYCUTER');

addpath([cuter_dir '/common/src/matlab']);

if nargin < 3
	sparam = '';
else
	sparam = sprintf('-param %s', param);
end

problem_name = upper(problem_name);

% try to call existing mex file
try
	prob = cuter_setup;
	rebuild = ~strcmp(prob.name(1:length(problem_name)), problem_name);
catch
	rebuild = 1;
end

if rebuild
	% (re)generate CUTEr mex file
%	eval(sprintf('!runcuter --package mx --decode %s/%s/%s.SIF %s %s', ...
%		sif_dir, testset, problem_name, sparam, '2> /dev/null > /dev/null'))

	cmd = sprintf('%s/bin/runcuter --package mx --decode %s/%s/%s.SIF %s', ...
		cuter_bin_dir, cuter_dir, testset, problem_name, sparam);
	
	evalc('system(cmd)');
%	system('ln -s mcuter.mexa64 mcuter.mex');

	% recheck
	prob = cuter_setup;
	sidx = find(prob.name ~= ' ', 1) - 1; % remove leading spaces
	if ~strcmp(prob.name(sidx+(1:length(problem_name))), problem_name)
		error('Could not setup problem "%s".', problem_name)
	end
end

