function test_cmaes()
%TEST_CMAES Not comprehensive. Just to make sure that the little changes
%I've made haven't screwed anything up.
tolerance = 1e-8;

theader('Testing cmaes.');

% Turn off file vomit.
cmaes_opts = cmaes;
% cmaes_opts.StopFitness = 1e-13;
% cmaes_opts.PopSize = 8;
cmaes_opts.TolFun = 1e-14;
cmaes_opts.TolHistFun = 1e-15;
cmaes_opts.StopOnStagnation = 'off';
cmaes_opts.LogPlot = 'off';
cmaes_opts.LogModulo = 0;
cmaes_opts.ReadSignals = 'off';
cmaes_opts.SaveVariables = 'off';
cmaes_opts.DispModulo = inf;
cmaes_opts.DispFinal = 'off';
cmaes_opts.Seed = 12345; % Make deterministic.

[xmin, fval] = cmaes(@rosenbrock, [0,0]', 2, cmaes_opts);

assert_near(xmin, [1;1], tolerance, 'CMAES did not find the minimum solution.');
assert_near(fval, 0, tolerance, 'CMAES did not find the minimum cost.');

%% ROSENBROCK(x) expects a two-column matrix and returns a column vector
% The output is the Rosenbrock function, which has a minimum at
% (1,1) of value 0, and is strictly positive everywhere else.
    function f = rosenbrock(x)
        x = x';
        f = 100*(x(:,2) - x(:,1).^2).^2 + (1 - x(:,1)).^2;
    end
end
