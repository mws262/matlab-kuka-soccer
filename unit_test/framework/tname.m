function tname(message)
validateattributes(message, {'string', 'char'}, {});
fprintf('Testing: %-25.25s... \t\t', message);
end