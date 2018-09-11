function setup(do_unit_tests)
% SETUP Adds all subfolders to the MATLAB path. Runs all unit tests unless
% specified.
%
%   SETUP(do_unit_tests);
%   SETUP();
%
%   Inputs:
%       `do_unit_tests` -- (OPTIONAL) true/false whether to perform all
%       unit tests. If this argument is not given, then the unit tests will
%       be run regardless.
%   Outputs: <none>
%
%   See also ADDPATH, TEST_ALL.
%

addpath ./
addpath ./vis/
addpath ./util/
addpath ./unit_test/
addpath ./tests/
addpath ./pushing_plane_versions/
addpath ./path_optim/
addpath ./iiwa_kinematics/
addpath ./geometry/
addpath ./figure_generation/
addpath ./dynamics/
addpath ./derived_autogen/
addpath ./data/
addpath ./unit_test/framework

% Runs all tests unless otherwise specified.
if nargin > 0
    validateattributes(do_unit_tests, {'logical'}, {'scalar'});
    if do_unit_tests
        test_all();
    end
else
    test_all();
end
end