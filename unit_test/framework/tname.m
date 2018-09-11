function tname(message)
% TNAME Prints a small header title for a sub-test.
%   
%   TNAME(message)
%
%   Inputs:
%       `message` -- Title of the sub-test.
%   Outputs: <none>
%
%   See also VALIDATEATTRIBUTES, ASSERT, ASSERT_NEAR, DO_TEST,
%   ASSERT_ERROR, THEADER, TNAME, REPORT_EXCEPTIONS.
%

validateattributes(message, {'string', 'char'}, {});
fprintf('Testing: %-25.25s... \t\t', message);
end