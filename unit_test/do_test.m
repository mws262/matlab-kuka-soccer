function except = do_test(testHandle)
%DO_TEST Summary of this function goes here
%   Detailed explanation goes here
except = false;
try
    testHandle();
catch except
    cprintf('*[0.8, 0.1, 0.1]', 'FAIL\n')
%     warning(except.message);
%     for i = 1:length(except.stack)
%        fprintf('%s, %s, %d \n', except.stack(i).file,except.stack(i).name,except.stack(i).line); 
%     end
    return;
end
cprintf('*[0.1, 0.8, 0.1]', 'PASS\n')
end

