function report_exceptions(exception_cell)
%REPORT_EXCEPTIONS Report all exceptions encountered while running the
%tests.

if ~isempty(exception_cell)
    fprintf('_______________________________________\n');
    cprintf('*red', '%d exception(s) to report.\n\n', numel(exception_cell));
    for i = 1:length(exception_cell)
            cprintf('*red', 'Exception %d: \n', i);
        disp( getReport(exception_cell{i}, 'extended', 'hyperlinks', 'on' ) );
        fprintf('\n\n');
    end
else
    fprintf('_______________________________________\n\n');
    cprintf('*[0.1, 0.8, 0.1]', 'ALL PASS\n')
end
end

