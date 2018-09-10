function theader(message)
%THEADER Summary of this function goes here
%   Detailed explanation goes here

validateattributes(message, {'string', 'char'}, {});

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
cprintf('*black', [message,'\n']);
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');

end

