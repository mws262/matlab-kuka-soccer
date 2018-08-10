clear all; close all;
%% Make the figure 8 spline.
% Makes a figure 8 cubic spline with periodic boundary conditions
% (continuous velocity and acceleration at the end to beginning
% transition).
addpath ./vis;
addpath ./derived_autogen

% System parameters.
I_num = 1;
m_num = 1;
R_num = 0.1;
course_offset = [0.35, 0, 0];
zero_accel_tol = 1e-2;
plane_tilt = 0/3;

tilt_rot = [1 0 0; 0 cos(plane_tilt) sin(plane_tilt); 0 -sin(plane_tilt) cos(plane_tilt)];

% Load a path previously acquired from optimization.
load_saved_path = true;
saved_path_name = 'path_data.mat';

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
[sphere_x,sphere_y,sphere_z] = sphere(25);
ball_patch = patch(surf2patch(R_num * sphere_x, R_num * sphere_y, R_num * sphere_z, 100*R_num * sphere_z));
ball_verts_untransformed = ball_patch.Vertices;
ball_patch.Vertices = ball_verts_untransformed + repmat(positions(1,:)  + [0, 0, R_num], [size(ball_verts_untransformed,1),1]);
ball_patch.FaceColor = 'interp';
ball_patch.EdgeAlpha = 0.0;
ball_patch.FaceAlpha = 1;

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

% Adds the arm to the world.
iiwa = IIWAImporter(scene_fig);


% lpatch = iiwa.link_patches{2};

geo_data = load('./data/iiwa_merged_end_effector.mat');
detail_level = 2;
faces = double(geo_data.merged_iiwa(detail_level).faces);
vertices = double(geo_data.merged_iiwa(detail_level).vertices);
face_normals = double(geo_data.merged_iiwa(detail_level).face_normals);
vertex_normals = double(geo_data.merged_iiwa(detail_level).vertex_normals);

merged_ee_patch = patch('Faces', faces, 'Vertices', vertices, 'FaceNormals', face_normals, 'VertexNormals', vertex_normals);
merged_patch_tform = hgtransform;
merged_ee_patch.Parent = merged_patch_tform;
tar_face = 1760;
cdata = zeros(size(merged_ee_patch.Faces));
cdata(:,1) = 1;
cdata(tar_face,:) = [0 1 0];
merged_ee_patch.FaceVertexCData = cdata;
merged_ee_patch.FaceColor = 'flat';
% merged_ee_patch.EdgeColor = 'None';
merged_ee_patch.LineWidth = 0.2;
merged_ee_patch.EdgeColor = [1, 0.8, 0.8];
merged_ee_patch.CDataMapping = 'direct';
[vtar1, vtar2, vtar3] = get_verts_from_face_idx(tar_face, faces, vertices);
surf_normal = face_normals(tar_face,:);
rel_target = double(vtar1);
% net_transform = transform_link_to_touch_surface_pt(rel_target, surf_normal, target_vec, target_normal, 1);
% iiwa.hgtransforms{2}.Matrix = net_transform;


hold off;

%% Pre-evaluate some stuff before animating and just interpolate in time later.
% Probably only really minor computational savings. Oh well.
zero_vec_pts = zeros(size(positions,1),1); % Zeros the length of tspan. Frequently used.

rotationQ = [1 0 0 0]; % Quaternion representing ball rotation.
w_num = [angular_rate_wx_fcn(R_num, velocities(:,2)), angular_rate_wy_fcn(R_num, velocities(:,1)), zero_vec_pts]; % World frame angular rate.
w_numQ = [zero_vec_pts, w_num]; % Angular rate with 0 in front for quaternion rotation.
arc_range = transpose(-pi/2:0.05:pi/2); % Range along the ball that the potential contact arc will be plotted.

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
    adjust_contact_locations_planar(tspan, v_surfx_eval, v_surfy_eval, accel_zero_break_start, accel_zero_break_end)

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

save('pps.mat', 'connecting_pps', 'accel_zero_break_start', 'accel_zero_break_end');

plane_x_offset_offset = -touch_section_ends(1,1);
plane_y_offset_offset = -touch_section_ends(1,2);
hold off;
%
% if ~isempty(connecting_pps)
link_name = 'iiwa_link_ee'; % Tip of ee. iiwa_link_ee is just the last full link.
home = iiwa.home_config;
guess = home;
%     numpts = 2;
%     tform_init = getIIWATForm(iiwa_vis, home, link_name);
% %     tform_way1 = trvec2tform(ppval(connecting_pps{1}, connecting_pps{1}.breaks(1):0.05:connecting_pps{1}.breaks(end))');
%     tform_way2 = trvec2tform(push_arc_center_world(:,:,1:1000:end));
%     %iiwa_configs = make_simple_trajectory(iiwa_vis, tform_init, tform_way1, home, link_name, numpts);
%     iiwa_configs = make_multi_point_trajectory(iiwa_vis, tform_way2, home, link_name);
%     iiwa_breaks = linspace(0, connecting_pps{1}.breaks(end), 2);
% end

%% Animation loop.
animation_speed_factor = 0.3;
update_arm = false;
linear_boundary_velocity_arrows_end.Visible = 'off';
linear_boundary_velocity_arrows_start.Visible = 'off';

% Video recording if desired
write_to_vid = false;
if write_to_vid
    framerate = 30;
    vid_writer = VideoWriter('make_8_vid.avi'); % Convert for Slack with 'ffmpeg -i infile.avi youroutput.mp4'
    vid_writer.FrameRate = framerate;
    vid_writer.Quality = 100;
    open(vid_writer);
end

lin_section_idx = 1;
in_linear_section = false;
tic;
prev_time = 0;
curr_time= 0;

while (curr_time < tspan(end)) && ishandle(scene_fig)
    h = curr_time - prev_time; % Time interval of this loop iteration.
    
    %     if ~isempty(connecting_pps)
    %         joint_pos_struct = interpolate_traj(iiwa_vis, iiwa_breaks, iiwa_configs, curr_time);
    %         display_at_pose(iiwa_vis, joint_pos_struct);
    %     end
    % Interpolate data for the current plotting time.
    wInterp = interp1(tspan, w_num, curr_time); % Angular rate.
    wQInterp = interp1(tspan, w_numQ, curr_time);
    
    positionsInterp = interp1(tspan, positions, curr_time);
    velsInterp = interp1(tspan, velocities, curr_time);
    accelsInterp = interp1(tspan, accelerations, curr_time);
    push_arc_center_interp = interp1(tspan, push_arc_center, curr_time);
    push_arc_center_world_interp = interp1(tspan, push_arc_center_world, curr_time);
    equator_contact_velocity_interp = interp1(tspan, equator_contact_velocity_eval, curr_time);
    surfx_integral_eval_interp = interp1(tspan, surfx_integral_eval, curr_time);
    surfy_integral_eval_interp = interp1(tspan, surfy_integral_eval, curr_time);
    
    surf_world_p_interp = interp1(tspan, p_surf_world, curr_time);
    
    % Integrate angular rate to get ball orientations. TODO: either
    % precalculate or at least improve integration
    rotationQ = 0.5 * quatprod(wQInterp, rotationQ) * h + rotationQ;
    rotationQ = rotationQ/norm(rotationQ);
    reverseQ = rotationQ .* [-1 1 1 1]; % MATLAB's reverse right-hand-rule for quaternions. Wow!
    ball_patch.Vertices = quatrotate(reverseQ, ball_verts_untransformed) + repmat(positionsInterp  + [0, 0, R_num], [size(ball_verts_untransformed,1),1]);
    
    
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
        
        
        
        
        net_transform = transform_link_to_touch_surface_pt(rel_target, surf_normal, tformed_offset, -R_norm_accel/R_num, 0);
        merged_patch_tform.Matrix = net_transform;
        
        if update_arm
            eetform = [[plane_rotation; 0, 0, 0], [(push_arc_center_world_interp' + 0.5* push_arc_center_interp'); 1]]; % Make 4x4 transformation out of rotation and translation.
            jnt_angs = single_ik_call(iiwa, eetform, guess, link_name);
            %jnt_angs = single_constrained_ik_call(iiwa_vis, eetform, guess, link_name);
            
            display_at_pose(iiwa, jnt_angs);
        end
    else
        %% Manipulator path is solely based on calculated COM trajectory. Does not need shifting.
        push_arc_plot.Visible = 'off'; % Don't need push/contact info.
        surface_vel_arrows.Visible = 'off';
        
        in_linear_section = true;
        for i=1:length(connecting_pps) % Check the polynomials to see if we're in range of any.
            curr_pp = connecting_pps(i);
            if curr_pp.breaks(1) - 1e-1  < curr_time && curr_time < curr_pp.breaks(end) + 1e-1
                
                % Tangent plane.
                ddx_start = ppval(poly_accel_x, curr_pp.breaks(1));
                ddy_start = ppval(poly_accel_y, curr_pp.breaks(1));
                ddx_end = ppval(poly_accel_x, curr_pp.breaks(end));
                ddy_end = ppval(poly_accel_y, curr_pp.breaks(end));
                
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
                
                
                
                
                
                
                
                net_transform = transform_link_to_touch_surface_pt(rel_target, surf_normal, move_spline_eval, ddx_tot_norm, 0);
                merged_patch_tform.Matrix = net_transform;
                
                if update_arm
                    %jnt_angs = single_constrained_ik_call(iiwa_vis, trvec2tform(move_spline_eval), guess, link_name);
                    
                    jnt_angs = single_ik_call(iiwa, trvec2tform(move_spline_eval), guess, link_name);
                    display_at_pose(iiwa, jnt_angs);
                end
                break;
            end
        end
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
