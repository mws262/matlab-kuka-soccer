function path_pp = get_path_pp(path_name, total_time)
% GET_PATH_PP Get a piecewise polynomial representing a path along the
% ground.
%
%   path_pp = GET_PATH_PP(path_name, total_time)
%
%   Inputs:
%       `path_name` -- Name of the path to return. Must be one of the
%       following:
%           large_arc -- Large arc with zero endpoint velocities.
%           small_arc -- Small arc with zero endpoint velocities.
%           small_arc_knot -- Small arc with not-a-knot end condition.
%           large_circle -- Large circle approximation.
%           triangle -- Smooth-cornered triangle made by test_qp_nlp_spline.
%           triangle_centered -- Smooth-cornered triangle made by 
%           test_qp_nlp_spline. Hand-centered about the origin.
%
%           `total_time` -- For splines which are made on the spot (i.e.
%           not loaded from file, this defines the total time spanned by
%           the breaks. TODO: Make this apply to all.
%       
%
%   See also SPLINE, CSAPE, NLP_SPLINE, QP_SPLINE, GET_MESH_DATA.
%

switch path_name
    case 'large_arc'
        R = 0.5;
        offset = [0.45;0;0];
        knots = [0,-R,0; R,0,0; 0, R,0]' + offset;
        breaks = linspace(0, total_time, size(knots,2));
        path_pp = spline(breaks, [[0;0;0], knots, [0;0;0]]);
        return;
    case 'small_arc'
        R = 0.3;
%         offset = [0.5;0.1;0];
        knots = [0,-0.5*R,0; 0.15*R,0,0; 0, 0.5*R,0]';% + offset;
        breaks = linspace(0, total_time, size(knots,2));
        path_pp = csape(breaks,  knots, 'not-a-knot');
        return;       
    case 'small_arc_knot'
        R = 0.1;
        offset = [0.4;0;0];
        knots = [0,R,0; R,0,0; 0, -R,0]' + offset;
        breaks = linspace(0, total_time, size(knots,2));
        path_pp = csape(breaks, knots, 'not-a-knot');
        return;
    case 'large_circle'
        R = 0.1;
        knots = [0,0,0; R+0.1,0,0; 0,2*R,0; -R-0.05,R + 0.07,0;0,0,0]' + [0.1,0.1,0]';
        breaks = linspace(0, total_time, size(knots,2));
        path_pp = csape(breaks, knots, 'periodic');
        return;
    case 'small_circle'
        R = 0.02;
        knots = [0,0,0; R,0,0; 0,2*R,0; -R,0,0]';
        breaks = linspace(0, total_time, size(knots,2));
        path_pp = csape(breaks, knots, 'periodic');
        return;
    case 'triangle'
        loaded_spl = load('triangle_position_spline.mat', 'triangle_spline');
        path_pp = loaded_spl.triangle_spline; 
        return;
    case 'triangle_centered'
        loaded_spl = load('triangle_position_spline_no_offset.mat', 'nlp_triangle_spline');
        path_pp = loaded_spl.nlp_triangle_spline;
        return;
    otherwise
        error('Unknown path_name given in the call to get a path piecewise polynomial.');
end
end

