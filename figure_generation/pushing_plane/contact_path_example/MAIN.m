%% Figure 8 example with "pushing plane" contact.
clear all; close all;
addpath ../../;
addpath ../../vis;
addpath ../../path_optim;
addpath ../../derived_autogen;
addpath ../../geometry;
addpath ../../dynamics;

% System parameters.
I_num = 1;
m_num = 1;
R_num = 0.1;
course_offset = [0.35, 0, 0]';
zero_accel_tol = 1e-2;
plane_tilt = 0;

tilt_rot = [1 0 0; 0 cos(plane_tilt) sin(plane_tilt); 0 -sin(plane_tilt) cos(plane_tilt)];

% Load a path previously acquired from optimization.
load_saved_path = true;
saved_path_name = '../../../data/path_data.mat';

% Load, rather than re-evaluated equation derivations.
rederive_equations = false;

%% Run or load derived equations.
if rederive_equations
    derived_eqns = do_derivations();
end

%% Run or load optimized path.
if exist(saved_path_name, 'file') && load_saved_path
    fprintf('Loading existing results from %s.\n', saved_path_name);
    load(saved_path_name);
else
    % Figure 8 parameters.
    segs_between = 3; % One of the most important parameters. Will change the number of extra segments between required knot points.
    time_scaling = 5; % Time to make one cycle along the 8.
    lobe_length = 0.5; % Parameters to stretch the 8.
    lobe_width = 0.2;
    lobe_center_offset = 0.0;
    height = 0; % Height of path off the ground. I see no reason to change this.
    
    % Points which must be passed through.
    knots = [0, 0, height;
        lobe_width, lobe_length/2 + lobe_center_offset, height;
        0, lobe_length, height;
        -lobe_width, lobe_length/2 + lobe_center_offset, height
        0, 0, height;
        lobe_width, -lobe_length/2 - lobe_center_offset, height;
        0, -lobe_length, height;
        -lobe_width, -lobe_length/2 - lobe_center_offset, height;
        0, 0, height]' - course_offset;
    
    num_knots = size(knots,2);
    % Times at each point.
    breaks = linspace(0, time_scaling, num_knots);
    [ppxqp, ppyqp] = qp_spline(breaks, knots', segs_between);
    [ppx, ppy] = nlp_spline(breaks, knots', segs_between, 1, ppxqp, ppyqp);
    poly_vel_x = fnder(ppx,1); % Derivative to get velocity along spline.
    poly_vel_y = fnder(ppy,1);
    poly_accel_x = fnder(poly_vel_x,1); % Derivative to get velocity along spline.
    poly_accel_y = fnder(poly_vel_y,1);
    total_breaks = ppx.breaks;
    
    tspan = linspace(0, breaks(end), 5000);
    positions = [ppval(ppx,tspan)', ppval(ppy, tspan)', zeros(size(tspan))'];
    velocities = [ppval(poly_vel_x,tspan)', ppval(poly_vel_y, tspan)', zeros(size(tspan))'];
    accelerations = [ppval(poly_accel_x,tspan)', ppval(poly_accel_y, tspan)', zeros(size(tspan))'];
    
    [accel_zero_break_start, accel_zero_break_end] = find_zero_accel_breaks(poly_accel_x, poly_accel_y, total_breaks, zero_accel_tol);
    
    save(saved_path_name, 'tspan', 'positions', 'velocities', 'accelerations', 'total_breaks', 'poly_vel_x', 'poly_vel_y', 'poly_accel_x', 'poly_accel_y', 'ppx', 'ppy', 'accel_zero_break_start', 'accel_zero_break_end');
end

%% Visualization.
scene_fig = make_visualizer_scene();

% Plot positions along spline.
plot(positions(:,1), positions(:,2), 'LineWidth', 1, 'Color', [0.1, 0.1, 0.1, 0.8]);

% Evaluate and plot forces along the spline.
f_vec_spacing = 30;
f_vec_scaling = 20;
applied_fx_eval = x_force_required_fcn(I_num, R_num, accelerations(:,1), m_num);
applied_fy_eval = y_force_required_fcn(I_num, R_num, accelerations(:,2), m_num);
quiver(positions(1:f_vec_spacing:end,1), positions(1:f_vec_spacing:end,2), f_vec_scaling*applied_fx_eval(1:f_vec_spacing:end), f_vec_scaling*applied_fy_eval(1:f_vec_spacing:end));

% Make the ball as a patch object.
[ball_patch, ball_verts_untransformed] = make_ball(R_num);
ball_patch.Vertices = ball_patch.Vertices + positions(1,:); % Move it to the start point.

% Arc on the ball representing possible places the arm could push.
push_arc_plot = plot(0,0,'g','LineWidth',5);

% Plane representing an object pushing the ball.
[plane_x, plane_y] = meshgrid(-0.5:0.1:0.5); % Generate x and y data
[plane_z] = zeros(size(plane_x, 1)); % Generate z data
plane_patch = patch(surf2patch(plane_x, plane_y, plane_z)); % Makes the plane, but the normal vector is in the z-direction.
plane_patch.FaceColor = [0.5,0.8,0.5];
plane_patch.EdgeColor = [0, 0, 1];
plane_patch.LineWidth = 3;
plane_patch.EdgeAlpha = 1;
plane_patch.FaceAlpha = 0.2;
plane_patch_verts = ([1, 0, 0; 0, 0, -1; 0 1 0]*plane_patch.Vertices')'; % Rotate the plane to align with the x axis (i.e. normal in y-direction).
plane_patch.Vertices = plane_patch_verts;
plane_center_dot = plot(0,0,'.g', 'MarkerSize', 20);

plane_x_offset = 0; % Shifting the visible part of the plane along the mathematical plane it represents.
plane_y_offset = 0;

% Surface velocity vector plot.
surface_vel_arrows = quiver(0,0,0,0);
surface_vel_arrows.Color = [1, 0, 0];
surface_vel_arrows.LineWidth = 2;

% Mark beginnings and ends of linear sections.
accel_zero_break_start = accel_zero_break_start - 1e-3; % The subtracted factor is to make sure that everything is evaluated NOT in the linear section. The velocity is continuous, but the point that we apply force at is not defined in the linear section.
accel_zero_break_end = accel_zero_break_end + 1e-3;
x_linear_start = ppval(ppx, accel_zero_break_start)';
y_linear_start = ppval(ppy, accel_zero_break_start)';
x_linear_end = ppval(ppx, accel_zero_break_end)';
y_linear_end = ppval(ppy, accel_zero_break_end)';

vx_linear_start = ppval(poly_vel_x, accel_zero_break_start)';
vy_linear_start = ppval(poly_vel_y, accel_zero_break_start)';
vx_linear_end = ppval(poly_vel_x, accel_zero_break_end)';
vy_linear_end = ppval(poly_vel_y, accel_zero_break_end)';

ax_linear_start = ppval(poly_accel_x, accel_zero_break_start)';
ay_linear_start = ppval(poly_accel_y, accel_zero_break_start)';
ax_linear_end = ppval(poly_accel_x, accel_zero_break_end)';
ay_linear_end = ppval(poly_accel_y, accel_zero_break_end)';


ball_linear_starts = contact_arc_fcn(R_num, ax_linear_start, ay_linear_start, x_linear_start, y_linear_start, plane_tilt * ones(size(y_linear_start)));
ball_linear_ends = contact_arc_fcn(R_num, ax_linear_end, ay_linear_end, x_linear_end, y_linear_end, plane_tilt * ones(size(y_linear_start)));

% TODO: make more sensible transpose conventions with the derived
% equations.
plot3(ball_linear_starts(:,1), ball_linear_starts(:,2), ball_linear_starts(:,3), '.', 'MarkerSize', 10, 'Color', [0 1 0]);
plot3(ball_linear_ends(:,1), ball_linear_ends(:,2), ball_linear_ends(:,3), '.', 'MarkerSize', 10, 'Color', [1 1 0]);

% Velocity at beginning and end of linear section arrows.
contact_vel_start_lin = world_contact_velocity_fcn(ax_linear_start', ay_linear_start', plane_tilt * ones(size(y_linear_start))', vx_linear_start', vy_linear_start')';
contact_vel_end_lin = world_contact_velocity_fcn(ax_linear_end', ay_linear_end', plane_tilt * ones(size(y_linear_start))', vx_linear_end', vy_linear_end')';

boundary_vel_arrow_scale = 0.5;
linear_boundary_velocity_arrows_start = quiver(0,0,0,0);
linear_boundary_velocity_arrows_start.AutoScale = 'off';
linear_boundary_velocity_arrows_start.XData = ball_linear_starts(:,1);
linear_boundary_velocity_arrows_start.YData = ball_linear_starts(:,2);
linear_boundary_velocity_arrows_start.ZData = ball_linear_starts(:,3);
linear_boundary_velocity_arrows_start.UData = boundary_vel_arrow_scale*contact_vel_start_lin(:,1);
linear_boundary_velocity_arrows_start.VData = boundary_vel_arrow_scale*contact_vel_start_lin(:,2);
linear_boundary_velocity_arrows_start.WData = boundary_vel_arrow_scale*contact_vel_start_lin(:,3);
linear_boundary_velocity_arrows_start.Color = [0,1,0];
linear_boundary_velocity_arrows_start.LineWidth = 1;

linear_boundary_velocity_arrows_end = quiver(0,0,0,0);
linear_boundary_velocity_arrows_end.AutoScale = 'off';
linear_boundary_velocity_arrows_end.XData = ball_linear_ends(:,1);
linear_boundary_velocity_arrows_end.YData = ball_linear_ends(:,2);
linear_boundary_velocity_arrows_end.ZData = ball_linear_ends(:,3);
linear_boundary_velocity_arrows_end.UData = boundary_vel_arrow_scale*contact_vel_end_lin(:,1);
linear_boundary_velocity_arrows_end.VData = boundary_vel_arrow_scale*contact_vel_end_lin(:,2);
linear_boundary_velocity_arrows_end.WData = boundary_vel_arrow_scale*contact_vel_end_lin(:,3);
linear_boundary_velocity_arrows_end.Color = [1,1,0];
linear_boundary_velocity_arrows_end.LineWidth = 1;
hold off;

%% Pre-evaluate some stuff before animating and just interpolate in time later.
% Probably only really minor computational savings. Oh well.
zero_vec_pts = zeros(size(positions,1),1); % Zeros the length of tspan. Frequently used.

rotationQ = [1 0 0 0]; % Quaternion representing ball rotation.
w_num = [angular_rate_wx_fcn(R_num, velocities(:,2)), angular_rate_wy_fcn(R_num, velocities(:,1)), zero_vec_pts]; % World frame angular rate.
arc_range = transpose(-pi/2:0.05:pi/2); % Range along the ball that the potential contact arc will be plotted.
quatspan = quatintegrate(tspan, rotationQ, w_num);
push_arc_center = contact_arc_centered_fcn(R_num, accelerations(:,1), accelerations(:,2),zero_vec_pts + plane_tilt); % NOT offset.
push_arc_center_world = contact_arc_fcn(R_num, accelerations(:,1), accelerations(:,2), positions(:,1), positions(:,2), zero_vec_pts + plane_tilt); % Offset. In world coordinates.

equator_contact_velocity_eval = equator_contact_velocity_fcn(accelerations(:,1), accelerations(:,2), velocities(:,1), velocities(:,2));
accel_vec_normalized = accelerations./sqrt(accelerations(:,1).^2 + accelerations(:,2).^2 + accelerations(:,3).^2);

v_surf_world = world_contact_velocity_fcn(accelerations(:,1)', accelerations(:,2)', plane_tilt, velocities(:,1)', velocities(:,2)')';
v_surfx_eval = v_surfx_fcn(accelerations(:,1), accelerations(:,2), plane_tilt, velocities(:,1), velocities(:,2));
v_surfy_eval = v_surfy_fcn(accelerations(:,1), accelerations(:,2), plane_tilt, velocities(:,1), velocities(:,2));

p_surf_world = cumtrapz(tspan, v_surf_world);


%% Adjust contact positions on the plane to minimize the total needed contact area.
[touch_section_starts, touch_section_ends, min_plane_x_dim, min_plane_y_dim] = ...
    adjust_contact_locations_planar(tspan, v_surfx_eval, v_surfy_eval, accel_zero_break_start, accel_zero_break_end);

