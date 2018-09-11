function assert_near(value1, value2, tolerance, msg)
% ASSERT_NEAR Check that two matrices have a maximum deviation from each
% other less than some tolerance.
%
%   ASSERT_NEAR(value1, value2, tolerance, msg)
%
%   Inputs:
%       `value1` -- First value to compare to. May be a scalar, a vector, 
%       or a matrix.
%       `value2` -- Second value to compare to. Must match the dimension of
%       value1.
%       `tolerance` -- Throws an exception if the maximum-element-deviation
%       is greater than this value.
%       `msg` -- Gives this error message if ASSERT_NEAR fails.
%   Outputs: <none>
%
%   See also VALIDATEATTRIBUTES, ASSERT, ASSERT_NEAR, DO_TEST,
%   ASSERT_ERROR, THEADER, TNAME, REPORT_EXCEPTIONS.
%

validateattributes(value1, {'numeric'}, {'real', 'nonnan'});
validateattributes(value2, {'numeric'}, {'real', 'nonnan', 'size', size(value1)});
validateattributes(tolerance, {'numeric'}, {'real', 'nonnegative'});
validateattributes(msg, {'string', 'char'}, {});

differance = value1 - value2;
assert(max(abs(differance(:))) < tolerance, msg);

end

