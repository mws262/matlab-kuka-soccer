function path_pp = get_path_pp(path_name, total_time)
%GET_PATH_PP Get a piecewise polynomial representing a path along the
%ground.
%   Valid names: 
% large_arc -- Large arc with zero endpoint velocities.
% small_arc -- Small arc with zero endpoint velocities.
% large_circle -- large circle approximation.

switch path_name
    case 'large_arc'
        R = 0.5;
        offset = [0.45;0;0];
        knots = [0,-R,0; R,0,0; 0, R,0]' + offset;
        breaks = linspace(0, total_time, size(knots,2));
        path_pp = spline(breaks, [[0;0;0], knots, [0;0;0]]);
        return;
    case 'small_arc'
        R = 0.1;
        offset = [0.45;0;0];
        knots = [0,-R,0; R,0,0; 0, R,0]' + offset;
        breaks = linspace(0, total_time, size(knots,2));
        path_pp = spline(breaks, [[0;0;0], knots, [0;0;0]]);
        return;
    case 'large_circle'
        R = 0.5;
        knots = [0,0,0; R,0,0; 0,2*R,0; -R,R,0;0,0,0]';
        breaks = linspace(0, total_time, size(knots,2));
        path_pp = csape(breaks, knots, 'periodic');
        return;
end
end

