function assert_near(value1, value2, tolerance, msg)
%ASSERT_NEAR Assert that one value or vector or matrix should have a
%maximum deviation from the other of tolerance.

validateattributes(value1, {'numeric'}, {'real', 'nonnan'});
validateattributes(value2, {'numeric'}, {'real', 'nonnan'});
validateattributes(tolerance, {'numeric'}, {'real', 'nonnegative'});
validateattributes(msg, {'string', 'char'}, {});

differance = value1 - value2;
assert(max(abs(differance(:))) < tolerance, msg);

end

