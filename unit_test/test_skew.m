function test_skew()
%TEST_SKEW Unit tests for skew.
%
%   TEST_SKEW()
%
%   Inputs: <none>
%   Outputs: <none>
%
%   See also SKEW, CROSS, TEST_ALL.
%

tolerance = 1e-15;

v1 = [-4.3, 6.8, 12.1]';
v2 = [3.4, -4, 0.1]';

theader('Testing skew');

assert_near(skew(v1)*v2, cross(v1,v2), tolerance, 'Skew multiply did not match cross product.');
assert_near(skew(v1')*v2, cross(v1,v2), tolerance, 'Skew multiply did not match cross product.');
assert_error(@()skew([1,2,3;5,4,2]));
assert_error(@()skew([1,2,3,4]));
assert_error(@()skew([1,2]));
assert_error(@()skew([1,2,3,4]'));
assert_error(@()skew([1,2]'));

end

