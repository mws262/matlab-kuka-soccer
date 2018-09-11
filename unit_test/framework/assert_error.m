function assert_error(expression_handle)
% ASSERT_ERROR Asserts that an error SHOULD occur. Throws if no error
% occurs. Does not check the type of exception produced.
%
%   Inputs:
%       `expresion_handle` -- Function handle to be tested. If it does not
%       produce an exception, then ASSERT_ERROR will throw its own
%       exception. If the function we are testing has its own arguments,
%       then the format will be something like @()foo(bar). The given
%       function hande should require no arguments.
%
%   Outputs: <none>
%
%   See also VALIDATEATTRIBUTES, ASSERT, ASSERT_NEAR, DO_TEST,
%   ASSERT_ERROR, THEADER, TNAME, REPORT_EXCEPTIONS.
%

assert(isa(expression_handle,'function_handle'), 'assert_error requires a function handle input');

exception_thrown = false;
try
    expression_handle();
catch except
    exception_thrown = true;
end

assert(exception_thrown, 'Exception was not thrown when one was expected.');

end