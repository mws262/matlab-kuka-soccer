close all;
clear all;

zero_accel_tol = 1e-5;
num_evaluation_pts = 400;

% Ball and scene
ball_radius = 0.1;
scene_fig = make_visualizer_scene();
[ball_patch, ball_verts_untransformed] = make_ball(ball_radius);

push_mesh = get_mesh_data('cube');
pos_pp = get_path_pp('triangle_centered');

[accel_zero_break_start, accel_zero_break_end, contact_polys, shifted_position_pp] = find_zero_accel_breaks_from_pos_pp(pos_pp, 3, zero_accel_tol);

[tspan, posspan, velspan, accelspan, omegaspan, quatspan, world_contact_loc_desired_fcn, contact_loc_desired_rel_com_fcn] = ...
    evaluate_spline(pos_pp, ball_radius, num_evaluation_pts);

draw_path_and_accel(posspan, accelspan, 1); % Draw out the path and acceleration arrows.

arc_angle = pi/4;
world_contact_desired_span = world_contact_loc_desired_fcn(arc_angle * ones(size(tspan)));
contact_desired_rel_com_span = contact_loc_desired_rel_com_fcn(arc_angle * ones(size(tspan)));

up_vector_span = contact_desired_rel_com_span/ball_radius;
surface_vel_span = cross(omegaspan, contact_desired_rel_com_span, 2); % Do we want a diff(contact_loc) term?

tspan_contact_idx = ...
    (tspan < accel_zero_break_start(1)) | ...
    (tspan > accel_zero_break_end(1) & tspan < accel_zero_break_start(2)) | ...
    (tspan > accel_zero_break_end(2) & tspan < accel_zero_break_start(3)) | ...
    (tspan > accel_zero_break_end(3));


%         contact_desired_rel_com_span = contact_loc_desired_rel_com_fcn(arc_angle * ones(size(tspan)));
plot3(world_contact_desired_span(tspan_contact_idx,1), world_contact_desired_span(tspan_contact_idx,2), world_contact_desired_span(tspan_contact_idx,3), '.', 'MarkerSize', 20);

int_vel_opt = integrate_velocity_over_surface('options');
int_vel_prob = integrate_velocity_over_surface('problem');
int_vel_prob.mesh_data = push_mesh;
int_vel_prob.time_vector = tspan;
int_vel_prob.initial_surface_point = [1, 0 ,0];
int_vel_prob.orientations_about_normal = 0;%ones(size(tspan)) * 0;
int_vel_prob.normals_to_match = up_vector_span;
int_vel_prob.velocity_vector = surface_vel_span;

inv_vel_out = integrate_velocity_over_surface(int_vel_prob, int_vel_opt);

figure;
plot3(inv_vel_out.mesh_surface_path(:,1), inv_vel_out.mesh_surface_path(:,2), inv_vel_out.mesh_surface_path(:,3));

% Make some interpolators for these values.
posspan_interpolator = general_interpolator(3);
posspan_interpolator.add_to_end_at_time(tspan, posspan);
velspan_interpolator = general_interpolator(3);
velspan_interpolator.add_to_end_at_time(tspan, velspan);
omegaspan_interpolator = general_interpolator(3);
omegaspan_interpolator.add_to_end_at_time(tspan, velspan);
quatspan_interpolator = general_interpolator(4);
quatspan_interpolator.add_to_end_at_time(tspan, quatspan);


tfactor = 1;
tic;
curr_time = 0;

while ishandle(scene_fig)
    
    quat_eval = interp1(tspan, quatspan, curr_time);
    pos_eval = interp1(tspan, posspan, curr_time);
    ball_patch.Vertices = quatrotate(quat_eval, ball_verts_untransformed) + repmat(pos_eval  + [0, 0, ball_radius], [size(ball_verts_untransformed,1),1]);
    
    
    
    surface_to_origin_translation = trvec2tform(-result_path_pts(i,:));
    world_origin_to_goal_pt_translation = trvec2tform(world_contact_desired_span(i,:));
    
    surf_vel_untransformed_to_planar_vel = rotm2tform(rot_integrated(:,:,i)');
    ball_contact_shift_tform = rotm2tform(get_rotation_from_vecs(up_vector_span(1,:), up_vector_span(i,:))); % upvec_rot(:,:,i));%
    manipulator_tform.Matrix = world_origin_to_goal_pt_translation * ball_contact_shift_tform * surf_vel_untransformed_to_planar_vel * surface_to_origin_translation;
    
    
    % Interpolate quantities in time.
    %     upvec_eval = up_vector_interp.get_at_time(curr_time);
    %     path_rot_eval = surface_path_rot_interp.get_at_time(curr_time);
    %     world_contact_pt_eval = world_contact_pt_interp.get_at_time(curr_time);
    %     surface_pts_eval = surface_path_pts_interp.get_at_time(curr_time);
    
    drawnow;
    
    curr_time = toc * tfactor;
    curr_time = mod(curr_time, tspan(end));
end