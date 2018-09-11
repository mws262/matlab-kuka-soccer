function report_exceptions(exception_cell)
% REPORT_EXCEPTIONS Reports all exceptions encountered while running unit
% tests. The purpose is to avoid throwing exceptions until all tests have
% run.
%
%   REPORT_EXCEPTIONS(exception_cell)
%
%   Inputs:
%       `exception_cell` -- Cell array with MException objects encountered
%       while running one or more unit tests.
%   Outputs: <none>
%
%   See also VALIDATEATTRIBUTES, ASSERT, ASSERT_NEAR, DO_TEST,
%   ASSERT_ERROR, THEADER, TNAME, REPORT_EXCEPTIONS.
%

validateattributes(exception_cell, {'cell'}, {});

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

