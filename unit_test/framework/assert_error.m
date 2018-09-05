function assert_error(expression_handle)
%ASSERT_ERROR Asserts that an error SHOULD occur.

exception_thrown = false;

try
    expression_handle();
catch except
    exception_thrown = true;
end

assert(exception_thrown, 'Exception was not thrown when one was expected.');

end