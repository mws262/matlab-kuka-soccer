function test_mesh_on_ball_multiseg_optim
close all; clear all;
addpath ../;
addpath ../vis/;
addpath ../geometry/;
addpath ../derived_autogen/;
addpath ../data/;
addpath ../path_optim/;

%% Import data for the dummy planning foot and the banned regions on it.
dummy_foot_dat = get_mesh_data('dummy_manipulator_mid_res');

%% Set up world scene.
% World scene.
scene_fig = make_visualizer_scene();

% Ball.
hold on;
ball_radius = 0.1;
[ball_patch, ball_verts_untransformed] = make_ball(ball_radius);

% Whole robot.
iiwa = IIWAImporter(scene_fig);

% Dummy floating foot (i.e. 'what should be happening').
dummy_foot_patch = patch('Faces', dummy_foot_dat.faces, 'Vertices', dummy_foot_dat.vertices, 'FaceNormals', dummy_foot_dat.face_normals, ...
    'VertexNormals', dummy_foot_dat.vertex_normals);
dummy_foot_patch.EdgeColor = 'none';
dummy_foot_patch.FaceColor = 'flat';
dummy_foot_patch.FaceVertexCData = repmat([0 0 1], [size(dummy_foot_patch.Faces,1),1]);
dummy_foot_patch.FaceAlpha = 0.3;
dummy_foot_patch.BackFaceLighting = 'lit';
dummy_foot_tform = hgtransform;
dummy_foot_patch.Parent = dummy_foot_tform;

campos([1.6970 1.5293 0.9466]);

%% Make goal path on the ground.
num_evaluation_pts = 150;
pos_pp = get_path_pp('triangle', []);
zero_accel_tol = 1e-4;

[accel_zero_break_start, accel_zero_break_end, contact_polys, shifted_position_pp] = find_zero_accel_breaks_from_pos_pp(pos_pp, 3, zero_accel_tol);

shifted_accel_pp = fnder(shifted_position_pp, 2);


[tspan, posspan, velspan, accelspan, omegaspan, quatspan, ...
    world_contact_loc_desired_fcn, contact_loc_desired_rel_com_fcn] = evaluate_spline(shifted_position_pp, ball_radius, num_evaluation_pts);

draw_path_and_accel(posspan, accelspan, 1); % Draw out the path and acceleration arrows.

zero_starts = ppval(pos_pp, accel_zero_break_start)';
zero_ends = ppval(pos_pp, accel_zero_break_end)';

% Make some interpolators for these values.
posspan_interpolator = general_interpolator(3);
posspan_interpolator.add_to_end(tspan, posspan);

velspan_interpolator = general_interpolator(3);
velspan_interpolator.add_to_end(tspan, velspan);

omegaspan_interpolator = general_interpolator(3);
omegaspan_interpolator.add_to_end(tspan, velspan);

quatspan_interpolator = general_interpolator(4);
quatspan_interpolator.add_to_end(tspan, quatspan);


% Plot the beginnings and ends of the 'ballistic' sections.
plot(zero_starts(:,1), zero_starts(:,2), '.g',  'MarkerSize', 20);
plot(zero_ends(:,1), zero_ends(:,2), '.r',  'MarkerSize', 20);

drawnow;


%% Split into a bunch of smaller problems for each contacting section.
% Initial guess.
% Solution for 1.
%     0.2146
%     5.9184
%     6.2652
%     1.1368
% Another, oops:
%     0.2118
%     5.8371
%     6.0726
%     1.3152
% %another?
% 0.2597
% 5.9063
% 6.2832
% 1.1076
% Solution for 2.
% initial_guess = [    0.1486
%     6.1941
%     5.7358
%     1.4480];
% 
% % sol, for 3
%     0.1427
%     2.8584
%     6.2832
%     1.3219

initial_guess = zeros(4,3);
initial_guess(:, 1) = [0.2146; 5.9184; 6.2652; 1.1368];
initial_guess(:, 2) = [0.1486; 6.1941; 5.7358; 1.4480];
initial_guess(:, 3) = [0.1427; 2.8584; 6.2832; 1.3219];


num_eval_pts = 50;
plot_flag = false;
just_evaluate_guess = true;

surface_path_pts_interp = general_interpolator(3);
surface_path_rot_interp = general_interpolator([3,3]);
world_contact_pt_interp = general_interpolator(3);
up_vector_interp = general_interpolator(3);

all_jnt_angles = {};
all_jnt_breaks = [];

for i = 1:length(contact_polys)
    [solution, jnt_angles_optim, breaks_optim, result_path_pts, path_rot_integrated, world_contact_desired_span, up_vector_span] = ...
        optimize_contact_region_kinematics(iiwa, initial_guess(:,i), contact_polys(i), num_eval_pts, ball_radius, plot_flag, just_evaluate_guess);
    section_tspan = linspace(contact_polys(i).breaks(1), contact_polys(i).breaks(end), num_eval_pts);
    
    surface_path_pts_interp.add_to_end_at_time(section_tspan, result_path_pts);
    surface_path_rot_interp.add_to_end_at_time(section_tspan, permute(path_rot_integrated,[3,1,2]));
    world_contact_pt_interp.add_to_end_at_time(section_tspan, world_contact_desired_span);
    up_vector_interp.add_to_end_at_time(section_tspan, up_vector_span);
    
    % Find a connector
    if i > 1
        curr_ctact_start_t = breaks_optim(1);
        connector_start_pos = ppval(shifted_position_pp, prev_ctact_endt);
        connector_end_pos = ppval(shifted_position_pp, curr_ctact_start_t);
        
        connector_start_accel = ppval(shifted_accel_pp, prev_ctact_endt - 0.1);
        connector_end_accel = ppval(shifted_accel_pp, curr_ctact_start_t + 0.1);
        
        connector_pos_diff = connector_end_pos - connector_start_pos;
        connector_unit_dir = connector_pos_diff/norm(connector_pos_diff);
        connector_unit_perp = cross(connector_unit_dir, [0,0,1]');
        
        connector_start_sign = -sign(dot(connector_start_accel, connector_unit_perp));
        connector_end_sign = -sign(dot(connector_end_accel, connector_unit_perp));
        
        if connector_start_sign * connector_end_sign == 1 % Same sign.
            connector_waypt = connector_pos_diff/2 + connector_start_pos + connector_unit_perp * 0.2 + [0;0;1] * 0.4;
        else % Opposite sign. Need to swerve above the ball.
            connector_waypt = connector_pos_diff/2 + connector_start_pos + [0;0;1] * 0.3;
            disp('above');
        end
        
        plot3(connector_waypt(1), connector_waypt(2), connector_waypt(3), '.b', 'MarkerSize', 20);
        
        % Turn off IK weighting on angle. Only want it at that position since
        % the point is to transition angles.
        iiwa.weights = [0 0 0 1 1 1];
        [connector_waypt_angles, solinfo] = single_ik_call(iiwa, trvec2tform(connector_waypt'), jnt_angles_optim{end}, 'iiwa_link_6');
        iiwa.weights = [0.5 0.5 0.5 1 1 1];
        
        % Add connecting waypoint solution.
        all_jnt_angles = [all_jnt_angles, connector_waypt_angles];
        all_jnt_breaks = [all_jnt_breaks; (curr_ctact_start_t - prev_ctact_endt) / 2 + prev_ctact_endt];
    end
    
    % Add the contact region.
    all_jnt_angles = [all_jnt_angles, jnt_angles_optim];
    all_jnt_breaks = [all_jnt_breaks; breaks_optim];
    prev_ctact_endt = breaks_optim(end);
    
end
%% Play the winning solution.
campos([1.6970    1.5293    0.9466]);
for i = 1:length(tspan) - 1
    
    ball_patch.Vertices = quatrotate(quatspan(i,:), ball_verts_untransformed) + repmat(posspan(i,:)  + [0, 0, ball_radius], [size(ball_verts_untransformed,1),1]);
    
    % Interpolate quantities in time.
    upvec_eval = up_vector_interp.get_at_time(tspan(i));
    path_rot_eval = surface_path_rot_interp.get_at_time(tspan(i));
    world_contact_pt_eval = world_contact_pt_interp.get_at_time(tspan(i));
    surface_pts_eval = surface_path_pts_interp.get_at_time(tspan(i));
    
%interp1(section_tspan(1:end-1), result_path_pts, tspan(i));   
%     first_idx = find(section_tspan <= tspan(i), 1, 'last');
%     last_idx = find(section_tspan >= tspan(i), 1, 'first');
%     path_rot_eval = (path_rot_integrated(:,:,last_idx) - path_rot_integrated(:,:,first_idx)) * (tspan(i) - section_tspan(first_idx)) + path_rot_integrated(:,:,first_idx);
%     path_rot_interp = interp1(eval_pt_tspan, path_rot_integrated, tspan(i));
    
%    if ~all(isnan(surface_pts_eval))
%        surface_to_origin_translation = trvec2tform(-surface_pts_eval);
%        world_origin_to_goal_pt_translation = trvec2tform(world_contact_pt_eval);
%        
%        surf_vel_untransformed_to_planar_vel = rotm2tform(squeeze(path_rot_eval)');
%        ball_contact_shift_tform = rotm2tform(get_rotation_from_vecs(up_vector_span(1,:), upvec_eval)); % upvec_rot(:,:,i));%
%        dummy_foot_tform.Matrix = world_origin_to_goal_pt_translation * ball_contact_shift_tform * surf_vel_untransformed_to_planar_vel * surface_to_origin_translation;
%    end
    
    joint_pos_struct = interpolate_traj(iiwa, all_jnt_breaks, all_jnt_angles, tspan(i));
    display_at_pose(iiwa, joint_pos_struct);
    
    drawnow;
    pause(0.001);
end


end