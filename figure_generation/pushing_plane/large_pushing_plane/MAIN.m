clear all; close all;
%% Make the figure 8 spline.
% Makes a figure 8 cubic spline with periodic boundary conditions
% (continuous velocity and acceleration at the end to beginning
% transition).

    % Figure 8 parameters.
    segs_between = 3; % One of the most important parameters. Will change the number of extra segments between required knot points.
    time_scaling = 5; % Time to make one cycle along the 8.
    lobe_length = 0.5; % Parameters to stretch the 8.
    lobe_width = 0.2;
    lobe_center_offset = 0.0;
    height = 0; % Height of path off the ground. I see no reason to change this.
    zero_accel_tol = 1e-3
    % Points which must be passed through.
    knots = [0, 0, height;
        lobe_width, lobe_length/2 + lobe_center_offset, height;
        0, lobe_length, height;
        -lobe_width, lobe_length/2 + lobe_center_offset, height
        0, 0, height;
        lobe_width, -lobe_length/2 - lobe_center_offset, height;
        0, -lobe_length, height;
        -lobe_width, -lobe_length/2 - lobe_center_offset, height;
        0, 0, height]'
num_knots = size(knots,2);
% Times at each point.
segs_between = 3;
breaks = linspace(0, time_scaling, num_knots);
% [ppxqp, ppyqp] = qp_spline(breaks, knots', segs_between);
% [ppx, ppy] = nlp_spline(breaks, knots', segs_between, 1, ppxqp, ppyqp);
    ppx = csape(breaks, knots(1,:), 'periodic');
    ppy = csape(breaks, knots(2,:), 'periodic');
poly_vel_x = fnder(ppx,1); % Derivative to get velocity along spline.
poly_vel_y = fnder(ppy,1);
poly_accel_x = fnder(poly_vel_x,1); % Derivative to get velocity along spline.
poly_accel_y = fnder(poly_vel_y,1);
total_breaks = ppx.breaks;

tspan = linspace(0, breaks(end), 1000);
positions = [ppval(ppx,tspan)', ppval(ppy, tspan)', zeros(size(tspan))'];
velocities = [ppval(poly_vel_x,tspan)', ppval(poly_vel_y, tspan)', zeros(size(tspan))'];
accelerations = [ppval(poly_accel_x,tspan)', ppval(poly_accel_y, tspan)', zeros(size(tspan))'];

%% Figure out where 0 acceleration sections begin and end. TODO simplify this stuff. too verbose.
max_accel_coef_x = max(abs(poly_accel_x.coefs),[],2); % For identifying regions where the acceleration goes to 0.
max_accel_coef_y = max(abs(poly_accel_y.coefs),[],2); % For identifying regions where the acceleration goes to 0.
accel_zero_segments = abs(max_accel_coef_x) + abs(max_accel_coef_y) < zero_accel_tol; % 1 if it is a zero-accel region.

start_zero_segs = false(length(accel_zero_segments));
start_zero_segs(1) = accel_zero_segments(1);
for i = 2:length(accel_zero_segments)
    if accel_zero_segments(i) && ~accel_zero_segments(i-1)
        start_zero_segs(i) = true;
    end
end

end_zero_segs = false(length(accel_zero_segments));
end_zero_segs(end) = accel_zero_segments(end);
for i = 1:length(accel_zero_segments) - 1
    if accel_zero_segments(i) && ~accel_zero_segments(i+1)
        end_zero_segs(i) = true;
    end
end

start_breaks = total_breaks(1:end-1);
end_breaks = total_breaks(2:end);

accel_zero_break_start = start_breaks(start_zero_segs);
accel_zero_break_end = end_breaks(end_zero_segs);

% poly_positions = csape(breaks, knots, 'periodic'); % Actually makes the spline.
% poly_velocities = fnder(poly_positions,1); % Derivative to get velocity along spline.
% poly_accelerations = fnder(poly_velocities,1); % Derivative to get acceleration along the spline.

% Evaluate many points along the spline for visualization.
% tspan = linspace(0, breaks(end), 1000);
% positions = ppval(poly_positions, tspan)';
% velocities = ppval(poly_velocities, tspan)';
% accelerations = ppval(poly_accelerations, tspan)';

%% Derive things about the dynamics.
syms g m I R fax fay faz fn ffx ffy rx ry vx vy ax ay wx wy wz wdx wdy wdz mu theta real;
% g - gravity
% m - ball mass
% I - ball inertia (Not using a matrix since it's a ball)
% R - ball radius
% fax - (scalar) force applied normal to the ball by the arm, xdir
% fay - (scalar) force applied normal to the ball by the arm, ydir
% faz - (scalar) force applied normal to the ball by the arm, zdir
% fn - (scalar) normal force from the ground
% ffx - (scalar) force from ground friction, x component
% ffy - (scalar) force from ground friction, y component
% vx - COM velocity in x
% vy - COM velocity in y
% ax - COM acceleration in x direction. Known from trajectory.
% ay - COM acceleration in y direction. Known from trajectory.
% wx - angular rate of ball about x axis
% wy - angular rate of ball about y axis
% wz - angular rate of ball about z axis
% wdx - angular acceleration of ball about x axis
% wdy - angular acceleration of ball about y axis
% wdz - angular acceleration of ball about z axis
% mu - ground to ball friction coefficient.
% theta - angle from horizontal along ball arc which force is applied

% X,Y,Z unit vectors
i = [1 0 0]';
j = [0 1 0]';
k = [0 0 1]';

Fn = fn*k; % Normal force is strictly vertical.
Ff = ffx*i + ffy*j; % Ground frictional forces are in the x-y plane.
Fg = -m*g*k; % Gravity is down.
Fa = fax*i + fay*j + faz*k; % Forces applied by the arm. Assumed to go directly through the COM (only forces applied in the normal direction by the arm).

Vcom = vx*i + vy*j; % Velocity of COM (known from trajectory).
Acom = ax*i + ay*j; % Acceleration of COM (known from trajectory).
w = wx*i + wy*j + wz*k; % Angular rate (world frame).
wd = wdx*i + wdy*j + wdz*k; % Angular acceleration (world frame).

rp_g = -R*k; % Ground contact point with respect to COM.

Fsum = Fn + Ff + Fg + Fa; % Sum of vector forces.
Tsum = cross(rp_g, Ff); % Sum of torques about the COM. Only ground friction does not go through the COM.

F_eqn = Fsum == m*Acom; % F = ma
T_eqn = Tsum == I*(-ay/R*i + ax/R*j); % Torque = I*alpha. I is a scalar because sphere.

% Planar forces are fully determined by trajectory. Vertical forces can be
% different.
[ffx_solve, ffy_solve, fax_solve, fay_solve] = solve(F_eqn(1:2), T_eqn(1:2), [ffx, ffy, fax, fay]) % Solve for frictional force and arm xy force components

Fnormal = solve(dot(Fsum, k), fn); % Total normal force.

% Friction cone requirement.
friction_required = sqrt(ffx_solve^2 + ffy_solve^2);
friction_max = Fnormal*mu;

force_xy_desired = sqrt(fax_solve.^2 + fay_solve.^2); % X and Y forces needed to do this motion.

force_xyz_magnitude = force_xy_desired/cos(theta); % Results in a different force magnitude based on position along the arc of the ball. We will need bigger forces if we are pressing farther away from the equator of the ball.

force_z_applied = -sin(theta) * force_xyz_magnitude; % And the total "downforce" depending on the position along the arc of the ball.

friction_actual_max = subs(friction_max, faz, force_z_applied);

% No slip at arm contact point gives actuator motion requirements.
no_slip_ground_eqn = cross(w,-rp_g) == Vcom;
[wx, wy, wz] = solve(no_slip_ground_eqn, w); % Find angular rate based on COM velocity. Note that wz stays zero under assumptions.

arm_contact_pt_equator = -R*Acom/norm(Acom); % Arm must push somewhere on the arc going from this point on the ball to the top and from this point to the bottom in world coordinates.
full_contact_arc = arm_contact_pt_equator*cos(theta) + R*sin(theta)*k; % Contact point can be on this arc for theta (-pi/2, pi/2) in world coordinates.
full_contact_arc_shifted = full_contact_arc + R*k + rx*i + ry*j; % Shift to position and height of ball.

equator_contact_velocity = cross([wx,wy,wz], arm_contact_pt_equator)% + [vx, vy, 0];

% Analytic pose integration?
% syms qw qx qy qz real;
% wq = [0, wx, wy, wz];
% q = [qw, qx, qy, qz];

% quatprod(wq, q)

%% Make functions for various symbolic equations (to avoid eval() confusion).
friction_required_fcn = matlabFunction(friction_required);
x_force_required_fcn = matlabFunction(fax_solve); % Required x component of force applied by arm.
y_force_required_fcn = matlabFunction(fay_solve);
angular_rate_wx_fcn = matlabFunction(wx);
angular_rate_wy_fcn = matlabFunction(wy);
contact_arc_fcn = matlabFunction(full_contact_arc_shifted');
contact_arc_centered_fcn = matlabFunction(full_contact_arc');
equator_contact_velocity_fcn = matlabFunction(equator_contact_velocity(3));


%% Visualization.
% Lazy evaluation of symbolic stuff -- plug in values.
I_num = 1;
m_num = 1;
R_num = 0.1;

scene_fig = figure;
hold on;

% Plot positions along spline.
plot(positions(:,1), positions(:,2), 'LineWidth', 1, 'Color', [0.1, 0.1, 0.1]);

% Evaluate and plot forces along the spline.
f_vec_spacing = 6;
f_vec_scaling = 3;
applied_fx_eval = x_force_required_fcn(I_num, R_num, accelerations(:,1), m_num);
applied_fy_eval = y_force_required_fcn(I_num, R_num, accelerations(:,2), m_num);
quiver(positions(1:f_vec_spacing:end,1), positions(1:f_vec_spacing:end,2), f_vec_scaling*applied_fx_eval(1:f_vec_spacing:end), f_vec_scaling*applied_fy_eval(1:f_vec_spacing:end));

% Make the ball as a patch object.
[sphere_x,sphere_y,sphere_z] = sphere(15);
ball_patch = patch(surf2patch(R_num * sphere_x, R_num * sphere_y, R_num * sphere_z, R_num * sphere_z));
ball_verts_untransformed = ball_patch.Vertices;
ball_patch.Vertices = ball_verts_untransformed + repmat(positions(1,:)  + [0, 0, R_num], [size(ball_verts_untransformed,1),1]);
ball_patch.FaceColor = 'interp';%[0.8,0.5,0.5];
ball_patch.EdgeAlpha = 0.2;
ball_patch.FaceAlpha = 1;

% Arc on the ball representing possible places the arm could push.
push_arc_plot = plot(0,0,'g','LineWidth',5);

% Plane representing an object pushing the ball.
[plane_x, plane_y] = meshgrid(-1:0.2:1); % Generate x and y data
[plane_z] = zeros(size(plane_x, 1)); % Generate z data
plane_patch = patch(surf2patch(plane_x, plane_y, plane_z)); % Makes the plane, but the normal vector is in the z-direction.
plane_patch.FaceColor = [0.3,0.3,0.3];
plane_patch.EdgeAlpha = 0.7;
plane_patch.FaceAlpha = 0.1;
plane_patch_verts = ([1, 0, 0; 0, 0, -1; 0 1 0]*plane_patch.Vertices')'; % Rotate the plane to align with the x axis (i.e. normal in y-direction).
plane_patch.Vertices = plane_patch_verts;

plane_x_offset = -1; % Shifting the visible part of the plane along the mathematical plane it represents.
plane_y_offset = 0;

% Floor plane representing surface that the ball is rolling on.
[floor_x, floor_y] = meshgrid(-10:0.5:10); % Generate x and y data
[floor_z] = zeros(size(floor_x, 1)); % Generate z data
floor_patch = patch(surf2patch(floor_x, floor_y, floor_z));
floor_patch.FaceColor = [0.8,0.8,0.6];
floor_patch.EdgeAlpha = 0.2;
floor_patch.FaceAlpha = 1;

% Surface velocity vector plot.
surface_vel_arrows = quiver(0,0,0,0);
surface_vel_arrows.Color = [1, 0, 0];
surface_vel_arrows.LineWidth = 2;

% Mark beginnings and ends of linear sections.
% accel_zero_break_start = accel_zero_break_start - 1e-3; % The subtracted factor is to make sure that everything is evaluated NOT in the linear section. The velocity is continuous, but the point that we apply force at is not defined in the linear section.
% accel_zero_break_end = accel_zero_break_end + 1e-3;
% x_linear_start = ppval(ppx, accel_zero_break_start)';
% y_linear_start = ppval(ppy, accel_zero_break_start)';
% x_linear_end = ppval(ppx, accel_zero_break_end)';
% y_linear_end = ppval(ppy, accel_zero_break_end)';
% 
% vx_linear_start = ppval(poly_vel_x, accel_zero_break_start)';
% vy_linear_start = ppval(poly_vel_y, accel_zero_break_start)';
% vx_linear_end = ppval(poly_vel_x, accel_zero_break_end)';
% vy_linear_end = ppval(poly_vel_y, accel_zero_break_end)';
% 
% ax_linear_start = ppval(poly_accel_x, accel_zero_break_start)';
% ay_linear_start = ppval(poly_accel_y, accel_zero_break_start)';
% ax_linear_end = ppval(poly_accel_x, accel_zero_break_end)';
% ay_linear_end = ppval(poly_accel_y, accel_zero_break_end)';

% ball_linear_starts = contact_arc_fcn(R_num, ax_linear_start, ay_linear_start, x_linear_start, y_linear_start, zeros(size(y_linear_start)));
% ball_linear_ends = contact_arc_fcn(R_num, ax_linear_end, ay_linear_end, x_linear_end, y_linear_end, zeros(size(y_linear_start)));
% 
% plot3(ball_linear_starts(:,1), ball_linear_starts(:,2), ball_linear_starts(:,3), '.', 'MarkerSize', 10, 'Color', [0 1 0]);
% plot3(ball_linear_ends(:,1), ball_linear_ends(:,2), ball_linear_ends(:,3), '.', 'MarkerSize', 10, 'Color', [1 1 0]);
% 
% % Velocity at beginning and end of linear section arrows.
% contact_vel_start_lin = equator_contact_velocity_fcn(ax_linear_start, ay_linear_start, vx_linear_start, vy_linear_start);
% contact_vel_end_lin = equator_contact_velocity_fcn(ax_linear_end, ay_linear_end, vx_linear_end, vy_linear_end);

% boundary_vel_arrow_scale = 0.5;
% linear_boundary_velocity_arrows_start = quiver(0,0,0,0);
% linear_boundary_velocity_arrows_start.AutoScale = 'off';
% linear_boundary_velocity_arrows_start.XData = ball_linear_starts(:,1);
% linear_boundary_velocity_arrows_start.YData = ball_linear_starts(:,2);
% linear_boundary_velocity_arrows_start.ZData = ball_linear_starts(:,3);
% linear_boundary_velocity_arrows_start.UData = boundary_vel_arrow_scale*contact_vel_start_lin(:,1);
% linear_boundary_velocity_arrows_start.VData = boundary_vel_arrow_scale*contact_vel_start_lin(:,2);
% linear_boundary_velocity_arrows_start.WData = boundary_vel_arrow_scale*contact_vel_start_lin(:,3);
% linear_boundary_velocity_arrows_start.Color = [0,1,0];
% linear_boundary_velocity_arrows_start.LineWidth = 1;
% 
% linear_boundary_velocity_arrows_end = quiver(0,0,0,0);
% linear_boundary_velocity_arrows_end.AutoScale = 'off';
% linear_boundary_velocity_arrows_end.XData = ball_linear_ends(:,1);
% linear_boundary_velocity_arrows_end.YData = ball_linear_ends(:,2);
% linear_boundary_velocity_arrows_end.ZData = ball_linear_ends(:,3);
% linear_boundary_velocity_arrows_end.UData = boundary_vel_arrow_scale*contact_vel_end_lin(:,1);
% linear_boundary_velocity_arrows_end.VData = boundary_vel_arrow_scale*contact_vel_end_lin(:,2);
% linear_boundary_velocity_arrows_end.WData = boundary_vel_arrow_scale*contact_vel_end_lin(:,3);
% linear_boundary_velocity_arrows_end.Color = [1,1,0];
% linear_boundary_velocity_arrows_end.LineWidth = 1;

% Make connecting splines.
% traj_col = cool(25);
% for i = 1:size(ball_linear_starts,1)
%     lin_st = ball_linear_starts(i,:);
%     lin_e = ball_linear_ends(i,:);
%     lin_perp = cross(lin_e - lin_st, k);
%     lin_perp = lin_perp/norm(lin_perp);
%     
%     lin_st_accel = [ax_linear_start(i), ay_linear_start(i), 0];
%     lin_e_accel = [ax_linear_end(i), ay_linear_end(i), 0];
%     way_dir_st = -sign(dot(lin_st_accel, lin_perp));
%     way_dir_e =  -sign(dot(lin_e_accel, lin_perp));
%     if way_dir_st*way_dir_e == 1 % Same sign. We want to scoop outwards around the ball.
%         lin_waypt = lin_perp*way_dir_st*0.2 + (lin_e + lin_st)/2 + 0.1*k';
%     else
%         lin_waypt = (lin_e + lin_st)/2 + 0.3*k'; % Different sign. Let's go over. We need to switch sides of the ball.
%     end
%     
%     lin_waypt_t = (accel_zero_break_start(i) + accel_zero_break_end(i))/2;
%     pp_connect = spline([accel_zero_break_start(i), lin_waypt_t, accel_zero_break_end(i)], [contact_vel_start_lin(i,:); lin_st; lin_waypt; lin_e; contact_vel_end_lin(i,:)]');
%     
%     eval_pp = ppval(pp_connect, linspace(accel_zero_break_start(i), accel_zero_break_end(i), 25))';
%     
% %     sp_pl = plot3(eval_pp(:,1), eval_pp(:,2), eval_pp(:,3),'b','LineWidth',1);
%     surf(repmat(eval_pp(:,1)',[3,1]), repmat(eval_pp(:,2)',[3,1]),repmat(eval_pp(:,3)',[3,1]), repmat(eval_pp(:,3)',[3,1]), 'EdgeColor','interp','LineWidth',1);
% %     drawnow;
% %     sp_pl.Edge.ColorBinding = 'interpolated';
% %     cd = [uint8(jet(25)*255) uint8(ones(25,1))].';
% %      sp_pl.Edge.ColorData = cd;%[interp1(linspace(1,101,100), traj_col, 100*eval_pp(:,3)/max(eval_pp(:,3))), ones(size(eval_pp(:,3)))];
%     sp_shadow = plot3(eval_pp(:,1), eval_pp(:,2), 0.001*ones(size(eval_pp,1),1),'LineWidth',1);
%     sp_shadow.Color = [0.1, 0.1, 0.1, 0.1];
% end

hold off;
% Whole scene settings.
axis([-3, 3, -3, 3]);
daspect([1,1,1]);
%shading faceted;
ax = scene_fig.Children;
ax.Projection = 'perspective';
ax.Clipping = 'off';
ax.Visible = 'off';
scene_fig.Position = [0, 0, 1500, 975];
ax.CameraPosition = [2, -3, 5];
ax.CameraTarget = [0, 0, 0];
% upvec = cross(ax.CameraTarget - ax.CameraPosition, j);
% ax.CameraUpVector = -upvec/norm(upvec);

camva(40);

zero_vec_pts = zeros(size(positions,1),1);

rotationQ = [1 0 0 0]; % Quaternion representing ball rotation.
w_num = [angular_rate_wx_fcn(R_num, velocities(:,2)), angular_rate_wy_fcn(R_num, velocities(:,1)), zero_vec_pts]; % World frame angular rate.
w_numQ = [zero_vec_pts, w_num]; % Angular rate with 0 in front for quaternion rotation.
arc_range = transpose(-pi/2:0.05:pi/2); % Range along the ball that the potential contact arc will be plotted.

push_arc_center = contact_arc_centered_fcn(R_num, accelerations(:,1), accelerations(:,2), zeros(size(accelerations(:,1)))); % NOT offset.
push_arc_center_world = contact_arc_fcn(R_num, accelerations(:,1), accelerations(:,2), positions(:,1), positions(:,2), zeros(size(accelerations(:,1)))); % Offset. In world coordinates.
equator_contact_velocity_eval = equator_contact_velocity_fcn(accelerations(:,1), accelerations(:,2), velocities(:,1), velocities(:,2));
accel_vec_normalized = accelerations./sqrt(accelerations(:,1).^2 + accelerations(:,2).^2 + accelerations(:,3).^2);

%% Animation loop.
laps_remaining = 1; % Doesn't work right here.
real_time_offset = 0;
prev_time = 0;
animation_speed_factor = 0.3;
tic;
prev_time = 0;
curr_time= 0;
campos([0 0 2]);

% Video recording if desired
write_to_vid = true;
if write_to_vid
    framerate = 30;
    vid_writer = VideoWriter(['../../../data/videos/raw/', 'pushing_plane_8_large.avi']); % Convert for Slack with 'ffmpeg -i infile.avi youroutput.mp4'
    vid_writer.FrameRate = framerate;
    vid_writer.Quality = 100;
    open(vid_writer);
end
% campos([0,-3,0.5])
while (curr_time < tspan(end))
    h = curr_time - prev_time; % Time interval of this loop.
    
    % Interpolate data for the current plotting time.
    wInterp = interp1(tspan, w_num, curr_time); % Angular rate.
    wQInterp = interp1(tspan, w_numQ, curr_time);
    
    positionsInterp = interp1(tspan, positions, curr_time);
    velsInterp = interp1(tspan, velocities, curr_time);
    accelsInterp = interp1(tspan, accelerations, curr_time);
    push_arc_center_interp = interp1(tspan, push_arc_center, curr_time);
    push_arc_center_world_interp = interp1(tspan, push_arc_center_world, curr_time);
    equator_contact_velocity_interp = interp1(tspan, equator_contact_velocity_eval, curr_time);
    
    % Integrate angular rate to get ball orientations. TODO: either
    % precalculate or at least improve integration
    rotationQ = 0.5 * quatprod(wQInterp, rotationQ) * h + rotationQ;
    rotationQ = rotationQ/norm(rotationQ);
    reverseQ = rotationQ .* [-1 1 1 1]; % MATLAB's reverse right-hand-rule for quaternions. Wow!
    ball_patch.Vertices = quatrotate(reverseQ, ball_verts_untransformed) + repmat(positionsInterp  + [0, 0, R_num], [size(ball_verts_untransformed,1),1]);
    
    if norm(accelsInterp) > zero_accel_tol
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
%         surface_vel_arrows.XData = push_arc_center_world_interp(1);
%         surface_vel_arrows.YData = push_arc_center_world_interp(2);
%         surface_vel_arrows.ZData = push_arc_center_world_interp(3);
%         surface_vel_arrows.UData = equator_contact_velocity_interp(1);
%         surface_vel_arrows.VData = equator_contact_velocity_interp(2);
%         surface_vel_arrows.WData = equator_contact_velocity_interp(3);
        
        % Tangent plane.
        center_ang_cos = dot(-R_norm_accel/R_num, [0; 1; 0]);
        center_ang_sin = dot(cross([0;1;0],-R_norm_accel/R_num),[0;0;1]);
        plane_rotation = [center_ang_cos, -center_ang_sin, 0; center_ang_sin, center_ang_cos, 0; 0 0 1];
        
        % TODO: I am suspicious of this. Make sure that all the offsets
        % integrate correctly without the plane being set tangent to the ball.
        plane_surf_vel = plane_rotation' * [0;0;equator_contact_velocity_interp]; % (cross(wInterp, world_push_arc_center_interp) + velsInterp)'; % World velocity of contact point to plane velocity.
        plane_x_offset = plane_x_offset + plane_surf_vel(1) * h; % Integrate tangent surface velocity.
        plane_y_offset = plane_y_offset + plane_surf_vel(2) * h;
        plane_patch.Vertices = (plane_rotation*(plane_patch_verts + repmat([plane_x_offset, 0, plane_y_offset], [size(plane_patch_verts,1),1]))')' + push_arc_center_world_interp;
        
    else
%         plane_patch.Visible = 'off';
%         push_arc_plot.Visible = 'off';
%         surface_vel_arrows.Visible = 'off';
% %         plane_x_offset = -1; % Reset the plane position.
% %         plane_y_offset = 0;
    end
    
    drawnow;
        prev_time = curr_time;
    if write_to_vid
        writeVideo(vid_writer, getframe(scene_fig));
        curr_time = 1./framerate * animation_speed_factor + curr_time;
    else
        curr_time = toc*animation_speed_factor;
    end

end

if write_to_vid
    close(vid_writer);
    % Convert from avi to mp4.
    !ffmpeg -i make_8_vid.avi make_8_vid.mp4
end