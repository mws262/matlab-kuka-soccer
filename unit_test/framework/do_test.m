function except = do_test(testHandle)
%DO_TEST Run tests, grade PASS/FAIL. Do not throw exceptions, just keep a
%list to rethrow later.
except = {};
try
    if nargout(testHandle) > 0 % If the test below this one can also throw exceptions, propagate these up the chain.
        sub_exceptions = testHandle();
        sub_exceptions = sub_exceptions(~cellfun('isempty', sub_exceptions)); % Get rid of any empty elements floating around.
        if ~isempty(sub_exceptions)
            except = sub_exceptions;
            cprintf('*[0.8, 0.1, 0.1]', 'FAIL\n');
            return;
        end
    else
        testHandle();
    end
    
catch ex
    cprintf('*[0.8, 0.1, 0.1]', 'FAIL\n');
    except = {ex};
    return;
end
cprintf('*[0.1, 0.8, 0.1]', 'PASS\n');
end

