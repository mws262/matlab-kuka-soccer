function [jnt_angles, breaks, solinfo, proj_start_pt, result_path_pts, path_rot_integrated, fail_flag] = attempt_kinematics_over_surface( ...
    point_to_project_to_surface, ... % Point on (or near) surface of dummy link. In dummy link coordinates. Can run the point picker to find these.
    placement_angle, ... % Angle to begin placement of the foot on the ball at.
    tspan, ... % Time interval to plan over.
    surface_vel, ... % Velocity at the surface of the ball where the foot is supposed to be.
    up_vectors, ... % Normal to the ball contact point. 
    world_contact_pt_over_time, ... % Contact point on the ball in world coords.
    link_dat_struct, ... % Data structure containing dummy link data. Has fields faces, vertices, face_normals, vertex_normals.
    num_kine_pts, ... % Number of points, equally distributed in time, to run inverse kinematics at.
    robot, ... % Robot object for running IK.
    ik_guess, ... % Initial guess for IK. Must be a special struct. Easiest to just use home position or a previous iteration result.
    kill_with_bad_sol, ... % Do we return without result if the velocity integration over the surface results in contact with banned regions of the dummy link?
    banned_regions) % Convex shape data for regions not allowed to use for contact. Run test_banned_regions for graphical explanation.

%% Project the given point down to the dummy surface.
[~, proj_start_pt, ~] = point2trimesh_with_normals( point_to_project_to_surface, link_dat_struct.faces, link_dat_struct.vertices,...
    link_dat_struct.face_normals, link_dat_struct.vertex_normals );

%% Integrate the given surface velocity over the dummy surface.
[result_path_pts, ~, path_rot_integrated, fail_flag] = ...
    integrate_velocity_over_surface(tspan, surface_vel, proj_start_pt, up_vectors, placement_angle, link_dat_struct, banned_regions);

if fail_flag
    jnt_angles = 0;
    breaks = 0;
    solinfo = 0;
    return;
end

%% Transform to arm's coordinates and make the knot pts
ik_idx = floor(linspace(1,length(tspan) - 1, num_kine_pts)); % Only doing IK for a smattering of pts along the full integrated path.
breaks = tspan(ik_idx);
knots = zeros(4,4,length(ik_idx));
for i = 1:length(ik_idx)
    ball_contact_shift_tform = rotm2tform(get_rotation_from_vecs(up_vectors(1,:), up_vectors(i,:)));
    knots(:,:,i) = dummy_link_tform_to_iiwa_model(path_rot_integrated(:,:,ik_idx(i)), ...
        result_path_pts(ik_idx(i),:), ball_contact_shift_tform, world_contact_pt_over_time(ik_idx(i),:));
end
[jnt_angles, solinfo] = make_multi_point_trajectory(robot, knots, ik_guess, 'iiwa_link_6', kill_with_bad_sol);
end