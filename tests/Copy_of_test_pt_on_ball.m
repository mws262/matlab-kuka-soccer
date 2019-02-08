function test_pt_on_ball
% Test mesh in contact with the ball as the ball rolls along a spline path.
% The contact location on the ball should obey the acceleration
% requirements. This is mostly to make sure that the geometry of the
% rolling contact is correct.

close all; clear all;

% Ball and scene
ball_radius = 0.1;
scene_fig = make_visualizer_scene();
[ball_patch, ball_verts_untransformed] = make_ball(ball_radius);

mesh_data = get_mesh_data('cube');
mesh_data.vertices = mesh_data.vertices .* [2, 0.2, 2]; % originally 0.1 HALF-extent.
cmap = flag(size(mesh_data.faces,1));

faces = mesh_data.faces;
vertices = mesh_data.vertices;
face_normals = mesh_data.face_normals;
vertex_normals = mesh_data.vertex_normals;

manipulator_patch = patch('Faces', faces, 'Vertices', vertices, 'FaceNormals', face_normals, 'VertexNormals', vertex_normals);
manipulator_patch.EdgeColor = [0.1, 0.1, 0.2];
manipulator_patch.FaceColor = 'flat';
manipulator_patch.FaceVertexCData = cmap;
manipulator_patch.FaceAlpha = 0.65;
manipulator_patch.BackFaceLighting = 'lit';
manipulator_tform = hgtransform;
manipulator_patch.Parent = manipulator_tform;

%% Pick a path polynomial.
path_pp = get_path_pp('large_circle', 5);

total_ts = 1000; % Total timesteps to evaluate at.
arc_angle = pi/2.7; % Angle along the possible arc of the ball to contact.

%% Evaluate ball and contact point quantities.
[tspan, posspan, velspan, accelspan, omegaspan, quatspan, ...
    world_contact_loc_desired_fcn, contact_loc_desired_rel_com_fcn, ~, jerkspan] = evaluate_spline(path_pp, ball_radius, total_ts);

world_contact_desired_span = world_contact_loc_desired_fcn(arc_angle*ones(size(tspan)));
contact_desired_rel_com_span = contact_loc_desired_rel_com_fcn(arc_angle*ones(size(tspan)));

vec1 = dot(contact_desired_rel_com_span(1,:)/ball_radius,[1,0,0]) * [1,0,0] + dot(contact_desired_rel_com_span(1,:)/ball_radius,[0,1,0]) * [0,1,0];
rot1 = get_rotation_from_vecs(vec1, contact_desired_rel_com_span(1,:)/ball_radius) * get_rotation_from_vecs([0;1;0], vec1);
quatinit = rotm2quat(rot1);

surface_vel_span = cross(omegaspan, contact_desired_rel_com_span, 2);

contact_pt_pl = plot(0,0,'.g','MarkerSize', 40);
integrated_contact_pt_pl = plot(0,0,'.y', 'MarkerSize', 40);

norm_out = contact_desired_rel_com_span(1,:)/ball_radius;
tan1 = cross([0,0,1], norm_out);
tan2 = cross(norm_out, tan1);
% init_pt = world_contact_desired_span(1,:) + norm_out * 0.02 + tan1 * 0.165 + tan2 * 0.11;
%init_pt = world_contact_desired_span(1,:) + norm_out * 0.02 + tan1 * -0.08 + tan2 * 0.00;
init_pt = world_contact_desired_span(1,:) + contact_desired_rel_com_span(1,:)/ball_radius * 0.02;


pos = path_pp;
vel = fnder(path_pp,1);
accel = fnder(vel,1);
jerk = fnder(accel,1);

function qdot = planeRHS(t, q)
    % Evaluate ball path polynomials.
    pos_eval = ppval(pos, t);
    vel_eval = ppval(vel, t);
    accel_eval = ppval(accel, t);
    jerk_eval = ppval(jerk, t);
    
    % Find linear velocity of the pushing surface.
    linear_vel = pusher_linear_velocity_fcn(ball_radius, accel_eval(1), accel_eval(2), ...
            jerk_eval(1), jerk_eval(2), q(1), q(2), q(3), ...
                pos_eval(1), pos_eval(2), 0, arc_angle, vel_eval(1), vel_eval(2))';
            
    % Find the angular velocity of the pushing surface.
    angular_vel = pusher_angular_velocity_fcn(accel_eval(1), accel_eval(2), jerk_eval(1), ...
        jerk_eval(2), 0, arc_angle);
    
    % verify -- Velocities at the contact point should match.
%     ball_surf_vel = world_contact_velocity_fcn(accel_eval(1), accel_eval(2), arc_angle, vel_eval(1), vel_eval(2));
%     ball_surf_vel2 = linear_vel + cross(angular_vel, contact_arc_fcn(ball_radius, accel_eval(1), accel_eval(2), pos_eval(1), pos_eval(2), arc_angle) - q(1:3)')';
%     ball_surf_vel2 - ball_surf_vel

    qdot = zeros(7,1);
    qdot(1:3) = linear_vel;
    qdot(4:7) = 0.5 * quatmultiply([0, angular_vel], q(4:7)');
end

options = odeset;
options.AbsTol = 1e-12;
options.RelTol = 1e-12;

[tarray, qarray] = ode45(@planeRHS, tspan, [init_pt, quatinit], options);
box_pos = qarray(:, 1:3);
box_quat = qarray(:, 4:7);

% box_accel = zeros(size(qarray,1), 6);
% for i = 1:size(qarray,1)
%     box_accel(i,:) = planeRHS(tarray(i), qarray(i,:)')'
% end



% Find linear velocity of the pushing surface.
box_vel = pusher_linear_velocity_fcn(ball_radius, accelspan(:, 1), accelspan(:, 2), ...
         jerkspan(:, 1), jerkspan(:, 2), box_pos(:, 1), box_pos(:, 2), box_pos(:, 3), ...
         posspan(:, 1), posspan(:, 2), 0, arc_angle, velspan(:, 1), velspan(:, 2));

% Find the angular velocity of the pushing surface.
box_omega = pusher_angular_velocity_fcn(accelspan(:, 1), accelspan(:, 2), jerkspan(:, 1), ...
        jerkspan(:, 2), 0, arc_angle);
    
box_accel = pusher_linear_acceleration_fcn(ball_radius, accelspan(:, 1), accelspan(:, 2), jerkspan(:, 1), jerkspan(:, 2), box_pos(:, 1), box_pos(:, 2), box_pos(:, 3),...
    posspan(:, 1), posspan(:, 2), 0, 0, 0, arc_angle, 0, velspan(:, 1), velspan(:, 2));

box_alpha = pusher_angular_acceleration_fcn(accelspan(:, 1), accelspan(:, 2), jerkspan(:, 1), jerkspan(:, 2), 0, 0, 0, arc_angle, 0);

draw_path_and_accel(posspan, accelspan, 3);

world_contact_desired_vel = surface_vel_span + velspan;

notes = 'Pushing box is a rectangular prism with xyz dimensions of 0.4, 0.04, 0.4. Untransformed, the box is centered about the origin. The ball radius is 0.1. Contact point positions are relative to the world origin.';
save_box_plan_to_file('box_oval', tspan, box_pos, box_quat, box_vel, box_omega, box_accel, box_alpha,...
    posspan + [0, 0, ball_radius], velspan, accelspan, quatspan, omegaspan, ...
    world_contact_desired_span, world_contact_desired_vel, ones(size(tspan)), notes);

% Check:
% (box_vel + cross(box_omega, world_contact_desired_span - box_pos, 2)) - ...
% (velspan + cross(omegaspan, world_contact_desired_span - (posspan + [0, 0, ball_radius]), 2))

% Contact pt agreement.
% ball_cent_ctact = (world_contact_desired_span - (posspan + [0,0, ball_radius]))/ball_radius;
% ctact_box_center = box_pos - world_contact_desired_span;
% b = max(0.02 - dot(ctact_box_center, ball_cent_ctact,2)) % should be 0.

for i = 1:1:length(tspan) - 1
    quat = quatspan(i,:);
    ball_patch.Vertices = quatrotate(quat, ball_verts_untransformed) + repmat(posspan(i,:)  + [0, 0, ball_radius], [size(ball_verts_untransformed,1),1]);
        
    % Show contact point.
    contact_pt_pl.XData = world_contact_desired_span(i,1);
    contact_pt_pl.YData = world_contact_desired_span(i,2);
    contact_pt_pl.ZData = world_contact_desired_span(i,3);
    
    integrated_contact_pt_pl.XData = box_pos(i,1);
    integrated_contact_pt_pl.YData = box_pos(i,2);
    integrated_contact_pt_pl.ZData = box_pos(i,3);
    
    % Surface velocity arrow.
    surface_vel_pl.XData = world_contact_desired_span(i,1);
    surface_vel_pl.YData = world_contact_desired_span(i,2);
    surface_vel_pl.ZData = world_contact_desired_span(i,3);
    surface_vel_pl.UData = surface_vel_span(i,1) + velspan(i,1);
    surface_vel_pl.VData = surface_vel_span(i,2) + velspan(i,2);
    surface_vel_pl.WData = surface_vel_span(i,3) + velspan(i,3);
    
    % Ball rotation axis arrow.
    omega_pl.XData = posspan(i,1);
    omega_pl.YData = posspan(i,2);
    omega_pl.ZData = posspan(i,3) + ball_radius;
    omega_pl.UData = omegaspan(i,1);
    omega_pl.VData = omegaspan(i,2);
    omega_pl.WData = omegaspan(i,3);
    
    manipulator_tform.Matrix = trvec2tform(box_pos(i,:))*quat2tform(box_quat(i,:));%*trvec2tform([0, 0.02, 0]);
    drawnow;
   % pause(0.05);
end


end
