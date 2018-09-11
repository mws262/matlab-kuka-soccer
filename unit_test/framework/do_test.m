function exceptions = do_test(test_handle)
% DO_TEST Run tests, grade PASS/FAIL. Do not throw exceptions, just keep a
% list to rethrow after all tests are done.
%
%   exceptions = DO_TEST(test_handle)
%
%   Inputs:
%       `test_handle` -- Function handle of a unit test to run.
%
%   Outputs:
%       `exceptions` -- A cell array of exceptions encountered while running
%       the unit test specified by test_handle.
%
%   See also VALIDATEATTRIBUTES, ASSERT, ASSERT_NEAR, DO_TEST,
%   ASSERT_ERROR, THEADER, TNAME, REPORT_EXCEPTIONS.
%


validateattributes(test_handle, {'function_handle'}, {});

exceptions = {};
try
    if nargout(test_handle) > 0 % If the test below this one can also throw exceptions, propagate these up the chain.
        sub_exceptions = test_handle();
        sub_exceptions = sub_exceptions(~cellfun('isempty', sub_exceptions)); % Get rid of any empty elements floating around.
        if ~isempty(sub_exceptions)
            exceptions = sub_exceptions;
            cprintf('*[0.8, 0.1, 0.1]', 'FAIL\n');
            return;
        end
    else
        test_handle();
    end
    
catch ex
    cprintf('*[0.8, 0.1, 0.1]', 'FAIL\n');
    exceptions = ex;
    return;
end
cprintf('*[0.1, 0.8, 0.1]', 'PASS\n');
end

