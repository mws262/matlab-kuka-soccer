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
plane_tilt = pi/6;

tilt_rot = [1 0 0; 0 cos(plane_tilt) sin(plane_tilt); 0 -sin(plane_tilt) cos(plane_tilt)];

% Load a path previously acquired from optimization.
load_saved_path = false;
saved_path_name = './path_data.mat';

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
    
%% QP version
qp_sol = qp_spline(breaks, knots', segs_between);

%% NLP version
nlp_problem = nlp_spline('problem');
nlp_problem.pinned_breaks = breaks;
nlp_problem.pinned_knots = knots';
nlp_problem.polys_per_pinned_knot = segs_between;
nlp_problem.guess_pp = qp_sol;
nlp_opts = nlp_spline('options');
nlp_opts.periodic_solutions = false;
% With no initial guess.
poly_pos = nlp_spline(nlp_problem, nlp_opts);
[accel_zero_break_start, accel_zero_break_end, contact_polys, shifted_position_pp] = find_zero_accel_breaks_from_pos_pp(poly_pos, 3, zero_accel_tol);

    poly_vel = fnder(poly_pos,1); % Derivative to get velocity along spline.
    poly_accel = fnder(poly_vel,1); % Derivative to get velocity along spline.
    total_breaks = poly_pos.breaks;
    
    tspan = linspace(0, breaks(end), 5000);
    positions = ppval(poly_pos,tspan)';
    velocities = ppval(poly_vel,tspan)';
    accelerations = ppval(poly_accel,tspan)';

    save(saved_path_name, 'tspan', 'positions', 'velocities', 'accelerations', 'total_breaks', 'poly_vel', 'poly_vel', 'poly_accel', 'poly_accel', 'poly_pos', 'accel_zero_break_start', 'accel_zero_break_end');
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
plane_patch.EdgeAlpha = 0.25;
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
% accel_zero_break_start = accel_zero_break_start;% - 1e-3; % The subtracted factor is to make sure that everything is evaluated NOT in the linear section. The velocity is continuous, but the point that we apply force at is not defined in the linear section.
% accel_zero_break_end = accel_zero_break_end + 1e-3;

linear_start = ppval(poly_pos, accel_zero_break_start)';
x_linear_start = linear_start(:,1);
y_linear_start = linear_start(:,2);
linear_end = ppval(poly_pos, accel_zero_break_end)';
x_linear_end = linear_end(:,1);
y_linear_end = linear_end(:,2);

v_linear_start = ppval(poly_vel, accel_zero_break_start)';
vx_linear_start = v_linear_start(:,1);
vy_linear_start = v_linear_start(:,2);
v_linear_end = ppval(poly_vel, accel_zero_break_end)';
vx_linear_end = v_linear_end(:,1);
vy_linear_end = v_linear_end(:,2);

a_linear_start = ppval(poly_accel, accel_zero_break_start)';
ax_linear_start = a_linear_start(:,1);
ay_linear_start = a_linear_start(:,2);
a_linear_end = ppval(poly_accel, accel_zero_break_end)';
ax_linear_end = a_linear_end(:,1);
ay_linear_end = a_linear_end(:,2);


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

%% Set the offset velocity sections to zero during the non-touch sections.
for i = 1:length(accel_zero_break_start)
    inactive_dx = find(accel_zero_break_start(i) >= tspan, 1, 'last'):find(accel_zero_break_end(i) >= tspan, 1, 'last');
    v_surfx_eval(inactive_dx) = 0;
    v_surfy_eval(inactive_dx) = 0;
end
surfx_integral_eval = cumtrapz(tspan, v_surfx_eval); % Evaluate the x coordinate integral of surface velocity, reflecting distance moved along the manipulator.
surfy_integral_eval = cumtrapz(tspan, v_surfy_eval);

% Switch back to main scene figure.
figure(1);
hold on;
% Change the pusher-plane size to match the minimum size needed for all the
% manuevers.
plane_patch.Vertices(:,1) = min_plane_x_dim .* plane_patch_verts(:,1);
plane_patch.Vertices(:,3) = min_plane_y_dim .* plane_patch_verts(:,3); % 3rd component because it's been rotated from its creation orientation.
plane_patch_verts = plane_patch.Vertices;


%% Make piecewise polynomials to bridge the gaps between contacts.
connecting_pps = ...
    make_contact_connecting_splines(accel_zero_break_start, accel_zero_break_end, ... % Times
    ball_linear_starts, ball_linear_ends, touch_section_starts, touch_section_ends, ... % Positions in world and on plane
    contact_vel_start_lin, contact_vel_end_lin, ... % Velocity boundary conditions.
    ax_linear_start, ay_linear_start,... % Needed accelerations at the beginning and end to apply the correct forces.
    ax_linear_end, ay_linear_end, plane_tilt);

for i = 1:size(ball_linear_starts,1)
    eval_pp = ppval(connecting_pps(i), linspace(accel_zero_break_start(i), accel_zero_break_end(i), 25))';
    connecting_spline_plot = plot3(eval_pp(:,1), eval_pp(:,2), eval_pp(:,3), 'LineWidth', 1.5); % Draw line projection downwards
    connecting_spline_plot.Color = [0.8, 0.8, 1, 0.9];
    connecting_spline_plot_shadows = plot3(eval_pp(:,1), eval_pp(:,2), 0.001*ones(size(eval_pp,1),1), 'LineWidth', 1); % Draw line projection downwards
    connecting_spline_plot_shadows.Color = [0.1, 0.1, 0.1, 0.1];
end

% save('../data/contact_bridging_pps.mat', 'connecting_pps', 'accel_zero_break_start', 'accel_zero_break_end');

plane_x_offset_offset = -touch_section_ends(1,1);
plane_y_offset_offset = -touch_section_ends(1,2);
hold off;

%% Animation loop.
animation_speed_factor = 1;
linear_boundary_velocity_arrows_end.Visible = 'off';
linear_boundary_velocity_arrows_start.Visible = 'off';
campos([-1.3721, -0.5317, 1.3582]);
camtarget([-0.1863, 0.0677, 0]);
% Video recording if desired
write_to_vid = false;
if write_to_vid
    framerate = 30;
    vid_writer = VideoWriter('make_8_vid.avi'); % Convert for Slack with 'ffmpeg -i infile.avi youroutput.mp4'
    vid_writer.FrameRate = framerate;
    vid_writer.Quality = 100;
    open(vid_writer);
end

%% Cam setup
laps_remaining = 5;
real_time_offset = 0;
prev_time = 0;
camtarget(-course_offset);
camposes = [    0.0433   -0.0065    1.5584;
        0.9371    0.4821    0.8333;
           -0.3406    1.4541    0.6847];
camtimes = [0; 3; 10];
    

lin_section_idx = 1;
in_linear_section = false;
tic;
prev_time = 0;
curr_time= 0;

while (curr_time < tspan(end)) && ishandle(scene_fig)
    h = curr_time - prev_time; % Time interval of this loop iteration.

    % Interpolate data for the current plotting time.
    quat_interp = interp1(tspan, quatspan, curr_time);
    positionsInterp = interp1(tspan, positions, curr_time);
    velsInterp = interp1(tspan, velocities, curr_time);
    accelsInterp = interp1(tspan, accelerations, curr_time);
    push_arc_center_interp = interp1(tspan, push_arc_center, curr_time);
    push_arc_center_world_interp = interp1(tspan, push_arc_center_world, curr_time);
    equator_contact_velocity_interp = interp1(tspan, equator_contact_velocity_eval, curr_time);
    surfx_integral_eval_interp = interp1(tspan, surfx_integral_eval, curr_time);
    surfy_integral_eval_interp = interp1(tspan, surfy_integral_eval, curr_time);
    
    surf_world_p_interp = interp1(tspan, p_surf_world, curr_time);
    ball_patch.Vertices = quatrotate(quat_interp, ball_verts_untransformed) + repmat(positionsInterp  + [0, 0, R_num], [size(ball_verts_untransformed,1),1]);
    
    
    if sum(curr_time >= accel_zero_break_start & curr_time <= accel_zero_break_end) == 0 % Not in between any zero acceleration breaks.
        if in_linear_section
            lift_diff = touch_section_starts(lin_section_idx + 1, :) - touch_section_ends(lin_section_idx, :);
            plane_x_offset_offset = plane_x_offset_offset + lift_diff(1);
            plane_y_offset_offset = plane_y_offset_offset + lift_diff(2);
            lin_section_idx = lin_section_idx + 1;
            in_linear_section = false;
        end
        
        plane_patch.Visible = 'on';
        push_arc_plot.Visible = 'on';
        surface_vel_arrows.Visible = 'on';
        
        R_norm_accel = -R_num*interp1(tspan, accel_vec_normalized, curr_time);
        
        % Make the arc along the ball on which we could apply the appropriate
        % force.
        world_push_arc = contact_arc_fcn(R_num, accelsInterp(1), accelsInterp(2), positionsInterp(1), positionsInterp(2), arc_range); % Offset by ball height and position.
        push_arc_plot.XData = world_push_arc(:,1);
        push_arc_plot.YData = world_push_arc(:,2);
        push_arc_plot.ZData = world_push_arc(:,3);
        
        % Velocity arrows
        surface_vel_arrows.XData = push_arc_center_world_interp(1);
        surface_vel_arrows.YData = push_arc_center_world_interp(2);
        surface_vel_arrows.ZData = push_arc_center_world_interp(3);
        surface_vel_arrows.UData = equator_contact_velocity_interp(1);
        surface_vel_arrows.VData = equator_contact_velocity_interp(2);
        surface_vel_arrows.WData = equator_contact_velocity_interp(3);
        
        % Tangent plane.
        center_ang_cos = dot(-R_norm_accel/R_num, [0; 1; 0]);
        center_ang_sin = dot(cross([0;1;0],-R_norm_accel/R_num),[0;0;1]);
        plane_rotation = [center_ang_cos, -center_ang_sin, 0; center_ang_sin, center_ang_cos, 0; 0 0 1];
        
        plane_x_offset = surfx_integral_eval_interp + plane_x_offset_offset;
        plane_y_offset = surfy_integral_eval_interp + plane_y_offset_offset;
        
        tformed_offset = plane_x_offset*isurf_fcn(accelsInterp(1), accelsInterp(2), plane_tilt) + ...
            plane_y_offset*jsurf_fcn(accelsInterp(1), accelsInterp(2), plane_tilt) + push_arc_center_world_interp;
        
        plane_patch.Vertices = (plane_rotation*tilt_rot*(plane_patch_verts)')' + tformed_offset;   
    else
        %% Manipulator path is solely based on calculated COM trajectory. Does not need shifting.
        push_arc_plot.Visible = 'off'; % Don't need push/contact info.
        surface_vel_arrows.Visible = 'off';
        
        in_linear_section = true;
        for i=1:length(connecting_pps) % Check the polynomials to see if we're in range of any.
            curr_pp = connecting_pps(i);
            if curr_pp.breaks(1) - 1e-1  < curr_time && curr_time < curr_pp.breaks(end) + 1e-1
                
                % Tangent plane.
                dd_start = ppval(poly_accel, curr_pp.breaks(1));
                ddx_start = dd_start(1);
                ddy_start = dd_start(2);
                dd_end = ppval(poly_accel, curr_pp.breaks(end));
                ddx_end = dd_end(1);
                ddy_end = dd_end(2);
                
                ddx_init = [ddx_start, ddy_start, 0];
                ddx_final = [ddx_end, ddy_end, 0];
                ddx_tot = (ddx_final - ddx_init)./(curr_pp.breaks(end) - curr_pp.breaks(1)) * (curr_time - curr_pp.breaks(1)) + ddx_init; % Just interpolating the orientation. It is determined on both sides by the required acceleration (or "push direction").
                ddx_tot_norm = ddx_tot/norm(ddx_tot);
                
                center_ang_cos = dot(ddx_tot_norm, [0; 1; 0]);
                center_ang_sin = dot(cross([0; 1; 0], ddx_tot_norm), [0; 0; 1]);
                plane_rotation = [center_ang_cos, -center_ang_sin, 0; center_ang_sin, center_ang_cos, 0; 0 0 1];
                
                move_spline_eval = ppval(curr_pp, curr_time)';
                
                plane_patch.Vertices = (plane_rotation*tilt_rot*(plane_patch_verts)')' + move_spline_eval;
                
                plane_center_dot.XData = move_spline_eval(1);
                plane_center_dot.YData = move_spline_eval(2);
                plane_center_dot.ZData = move_spline_eval(3);  
                
                break;
            end
        end
    end
    
    %% Cam update
    campose_current = interp1(camtimes, camposes, curr_time + real_time_offset, 'linear', 0);
    if max(abs(campose_current)) ~= 0
    campos(campose_current);
    end
    
    drawnow;
    prev_time = curr_time;
    if write_to_vid
        writeVideo(vid_writer, getframe(scene_fig));
        curr_time = 1./framerate * animation_speed_factor + curr_time;
    else
        curr_time = toc*animation_speed_factor;
    end
    
    curr_time = mod(curr_time, tspan(end));
    if curr_time < prev_time
        laps_remaining = laps_remaining - 1;
        real_time_offset = real_time_offset + tspan(end);
        lin_section_idx = 1;
        if laps_remaining == 0;
            break;
        end
    end
    prev_time = curr_time;
end

if write_to_vid
    close(vid_writer);
    % Convert from avi to mp4.
    !ffmpeg -i make_8_vid.avi make_8_vid.mp4
end
