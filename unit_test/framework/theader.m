function theader(message)
%THEADER Prints a big header for the beginning of a set of unit tests.
%   
%   THEADER(message)
%
%   Inputs:
%       `message` -- Title of the unit test.
%   Outputs: <none>
%
%   See also VALIDATEATTRIBUTES, ASSERT, ASSERT_NEAR, DO_TEST,
%   ASSERT_ERROR, THEADER, TNAME, REPORT_EXCEPTIONS.
%

validateattributes(message, {'string', 'char'}, {});

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
cprintf('*black', [message,'\n']);
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');

end

