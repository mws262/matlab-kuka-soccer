% Test mesh in contact with the ball as the ball rolls along a spline path.
% The contact location on the ball should obey the acceleration
% requirements. This is mostly to make sure that the geometry of the
% rolling contact is correct.

close all; clear all;

% Ball and scene
ball_radius = 0.1;
scene_fig = make_visualizer_scene();
[ball_patch, ball_verts_untransformed] = make_ball(ball_radius);

%% Pick a path polynomial.
path_pp = get_path_pp('large_circle', 5);

total_ts = 250; % Total timesteps to evaluate at.
approach_ang = 0;
arc_angle = pi/2.5; % Angle along the possible arc of the ball to contact.
initial_surface_point = [0,1,0]; % Initial point on the surface to project down.

%% Evaluate ball and contact point quantities.
[tspan, posspan, velspan, accelspan, omegaspan, quatspan, ...
    world_contact_loc_desired_fcn, contact_loc_desired_rel_com_fcn, contact_loc_world_vel] = evaluate_spline(path_pp, ball_radius, total_ts);

world_contact_desired_span = world_contact_loc_desired_fcn(arc_angle*ones(size(tspan)));
contact_desired_rel_com_span = contact_loc_desired_rel_com_fcn(arc_angle*ones(size(tspan)));
contact_loc_vel = contact_loc_world_vel(arc_angle*ones(size(tspan)), zeros(size(tspan)));

up_vector_span = contact_desired_rel_com_span/ball_radius;
surface_vel_span = cross(omegaspan, contact_desired_rel_com_span, 2);

contact_pt_pl = plot(0,0,'.g','MarkerSize', 40);
integrated_contact_pt_pl = plot(0,0,'.y', 'MarkerSize', 40);
init_pt = world_contact_desired_span(1,:);

integrated_pt_span = cumtrapz(tspan(2:end-1), contact_loc_vel(2:end-1,:)) + init_pt;

for i = 1:length(tspan) - 1
    quat = quatspan(i,:);
    ball_patch.Vertices = quatrotate(quat, ball_verts_untransformed) + repmat(posspan(i,:)  + [0, 0, ball_radius], [size(ball_verts_untransformed,1),1]);
        
    % Show contact point.
    contact_pt_pl.XData = world_contact_desired_span(i,1);
    contact_pt_pl.YData = world_contact_desired_span(i,2);
    contact_pt_pl.ZData = world_contact_desired_span(i,3);
    
    integrated_contact_pt_pl.XData = integrated_pt_span(i,1);
    integrated_contact_pt_pl.YData = integrated_pt_span(i,2);
    integrated_contact_pt_pl.ZData = integrated_pt_span(i,3);
    
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
    drawnow;
    pause(0.05);
end



