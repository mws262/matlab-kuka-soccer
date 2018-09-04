function do_test(testHandle)
%DO_TEST Summary of this function goes here
%   Detailed explanation goes here
try
    testHandle();
catch except
    disp('FAILED');
    warning(except.message);
    for i = 1:length(except.stack)
       fprintf('%s, %s, %d \n', except.stack(i).file,except.stack(i).name,except.stack(i).line); 
    end
    return;
end
disp('PASSED');
end

