function path_pp = get_path_pp(path_name, total_time)
%GET_PATH_PP Get a piecewise polynomial representing a path along the
%ground.
%   Valid names:
% large_arc -- Large arc with zero endpoint velocities.
% small_arc -- Small arc with zero endpoint velocities.
% small_arc_knot -- Small arc with not-a-knot end condition.
% large_circle -- Large circle approximation.
% triangle -- Smooth-cornered triangle made by test_qp_nlp_spline.

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
    case 'small_arc_knot'
        R = 0.1;
        offset = [0.4;0;0];
        knots = [0,R,0; R,0,0; 0, -R,0]' + offset;
        breaks = linspace(0, total_time, size(knots,2));
        path_pp = csape(breaks, knots, 'not-a-knot');
        return;
    case 'large_circle'
        R = 0.5;
        knots = [0,0,0; R,0,0; 0,2*R,0; -R,R,0;0,0,0]';
        breaks = linspace(0, total_time, size(knots,2));
        path_pp = csape(breaks, knots, 'periodic');
        return;
    case 'triangle'
        loaded_spl = load('triangle_position_spline.mat', 'triangle_spline');
        path_pp = loaded_spl.triangle_spline; % THIS IGNORES THE TIME SCALING BECAUSE IT MESSES STUFF UP. TODO.
%         time_scale = total_time/path_pp.breaks(end); % Rescale the timing.
%         path_pp.breaks = path_pp.breaks * time_scale; 
        return;
    otherwise
        error('Unknown path_name given in the call to get a path piecewise polynomial.');
end
end

