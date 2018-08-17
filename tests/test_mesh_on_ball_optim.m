function test_mesh_on_ball_optim
close all; clear all;
addpath ../;
addpath ../vis/;
addpath ../geometry/;
addpath ../derived_autogen/;
addpath ../data/;
addpath ../path_optim/;


cyl_fig = figure(102);
hold on;
% Manipulator mesh
detail_level = 2;
geo_data = load('../data/iiwa_merged_end_effector.mat');
banned = geo_data.banned_regions;
picked_pts = load('../data/picked_faces_single_end.mat', 'selections');
faces = geo_data.merged_iiwa(detail_level).faces;
vertices = geo_data.merged_iiwa(detail_level).vertices;
face_normals = geo_data.merged_iiwa(detail_level).face_normals;
vertex_normals = geo_data.merged_iiwa(detail_level).vertex_normals;
iiwa_patch = patch('Faces', faces, 'Vertices', vertices, 'FaceNormals', face_normals, 'VertexNormals', vertex_normals);
iiwa_patch.EdgeColor = 'none';%[0.1, 0.1, 0.1];
iiwa_patch.FaceColor = 'flat';
iiwa_patch.FaceVertexCData = repmat([0 0 1], [size(iiwa_patch.Faces,1),1]);
iiwa_patch.FaceAlpha = 0.8;
iiwa_patch.BackFaceLighting = 'lit';
iiwa_tform = hgtransform;
iiwa_patch.Parent = iiwa_tform;


maxx = max(iiwa_patch.Vertices(:,1),[],1);
minx = min(iiwa_patch.Vertices(:,1),[],1);
maxy = max(iiwa_patch.Vertices(:,2),[],1);
miny = min(iiwa_patch.Vertices(:,2),[],1);
maxz = max(iiwa_patch.Vertices(:,3),[],1);
minz = min(iiwa_patch.Vertices(:,3),[],1);

% longitudinal axis aligned with x.
cyl_rad = max(maxy - miny, maxz - minz)/2;
cyl_off_y = (miny + maxy)/2;
cyl_off_z = (minz + maxz)/2;
cyl_off_x = minx;
cyl_height = maxx - minx;

[y_cyl, z_cyl, x_cyl] = cylinder(cyl_rad, 20);
cyl_patch = patch(surf2patch(x_cyl, y_cyl, z_cyl, 'triangles'))
cyl_patch.Vertices(:,1) = cyl_patch.Vertices(:,1)*cyl_height + cyl_off_x;
cyl_patch.Vertices(:,2) = cyl_patch.Vertices(:,2) + cyl_off_y;
cyl_patch.Vertices(:,3) = cyl_patch.Vertices(:,3) + cyl_off_z;
cyl_patch.FaceAlpha = 0.3;
min_ang = 0;
max_ang = 2*pi;
cyl_h_ext = 0.05;
low_line = plot3([cyl_off_x - cyl_h_ext, cyl_off_x + cyl_height + cyl_h_ext], [cyl_rad*cos(min_ang), cyl_rad*cos(min_ang)] + cyl_off_y, [cyl_rad*sin(min_ang), cyl_rad*sin(min_ang)] + cyl_off_z, '-g', 'LineWidth', 2);
high_line = plot3([cyl_off_x - cyl_h_ext, cyl_off_x + cyl_height + cyl_h_ext], [cyl_rad*cos(max_ang), cyl_rad*cos(max_ang)] + cyl_off_y, [cyl_rad*sin(max_ang), cyl_rad*sin(max_ang)] + cyl_off_z, '-r', 'LineWidth', 2);

% [proj_height + cyl_off_x, cyl_rad*cos(proj_ang) + cyl_off_y, cyl_rad*sin(proj_ang) + cyl_off_z]

xlabel('x');
ylabel('y');
zlabel('z');
view(3);
daspect([1,1,1]);
camlight HEADLIGHT;
hold off;

% Ball and scene
radius = 0.1; %BALL radius
scene_fig = make_visualizer_scene();
hold on;
[ball_patch, ball_verts_untransformed] = make_ball(radius);
% Whole robot.
iiwa = IIWAImporter(scene_fig);
link_name = 'iiwa_link_6'; % Tip of ee. iiwa_link_ee is just the last full link.
home = iiwa.home_config;
guess = home;

% More interesting path.
tend = 5;
laps = 1;
R = 0.1; % path radius
offset = [0.45;0;0];
% knots = [0,0,0; R,R,0; 0,2*R,0; -R,R,0;0,0,0]' + offset;
knots = [0,-R,0; R,0,0; 0, R,0]' + offset;

%knots = [0,0,0; 2*R,0,0;-R,0,0; 0,0,0]' + offset;
breaks = linspace(0, tend, size(knots,2));

% Make the simple path spline.
pos_pp = csape(breaks, knots, 'not-a-knot');


% Arc
R = 0.1;
offset = [0.4;0;0];
knots = [0,-R*1.5,0; R/2,0,0; 0, R*1.5,0]' + offset;

breaks = linspace(0, tend, size(knots,2));
pos_pp = spline(breaks, [[0;0;0], knots, [0;0;0]]);


vel_pp = fnder(pos_pp);
accel_pp = fnder(vel_pp);
pts_per_lap = 125;
tspan_eval = linspace(0, tend, pts_per_lap)';
% Replicate everything if we want to do multiple laps.
pos_eval = repmat(ppval(pos_pp, tspan_eval)',[laps,1]);
vel_eval = repmat(ppval(vel_pp, tspan_eval)',[laps,1]);
accel_eval = repmat(ppval(accel_pp, tspan_eval)',[laps,1]);
tspan = linspace(0, tend*laps, pts_per_lap*laps)';
hold on;
plot(pos_eval(:,1), pos_eval(:,2), 'LineWidth', 2);


% States, etc
zero_vec_pts = zeros(size(tspan));
omega = [angular_rate_wx_fcn(radius, vel_eval(:,2)), angular_rate_wy_fcn(radius, vel_eval(:,1)), zero_vec_pts];

rotationQ = [1 0 0 0]; % Quaternion representing ball rotation.
% contact_loc = repmat([0 0 2*radius],[length(tspan), 1]);
% contact_loc_over_time = contact_loc + pos_eval;
% surface_vel = cross(omega, contact_loc, 2);

ball_patch.Vertices = ball_verts_untransformed + repmat(pos_eval(1,:)  + [0, 0, radius], [size(ball_verts_untransformed,1),1]);

ik_idx = floor(linspace(1,length(tspan) - 1, 5*laps)); % Sample some indices from the whole tspan to use for IK.
iiwa_breaks = tspan(ik_idx);
iiwa_knots = zeros(4, 4, length(ik_idx));


cmap_jet = jet(12);
        figure(102);
        hold on;
cmaes_opts = cmaes;
cmaes_opts.LogPlot = 'off';
cmaes_opts.SaveVariables = 'off';
cmaes_opts.UBounds = [cyl_height + cyl_h_ext, max_ang, 2*pi, pi/2-0.03]';
cmaes_opts.LBounds = [-cyl_h_ext, min_ang, 0, 0]';
cmaes_opts.StopFitness = 5e-8;
cmaes_opts.MaxFunEvals = 300;
cmaes_opts.PopSize = 8;
cma_init = [    0.1060
    3.5018
    2.4155
    0.5668];
[xres, fval] = cmaes(@cost_fun_wrap, cma_init, 0.2*(cmaes_opts.UBounds - cmaes_opts.LBounds), cmaes_opts)


    function err = cost_fun_wrap(X)
        err = cost_fun(X(1,:), X(2,:), X(3,:),X(4,:), 10, true);
    end
    function [error, jnt_angles_optim, breaks_optim, solinfo_optim, result_path_pts, path_rot_integrated] = cost_fun(proj_height, proj_ang, placement_ang, arc_angle, num_pts, kill_on_bad_sol)
        point_to_project_to_surface = [proj_height + cyl_off_x, cyl_rad*cos(proj_ang) + cyl_off_y, cyl_rad*sin(proj_ang) + cyl_off_z];

        contact_loc_over_time = contact_arc_fcn(radius, accel_eval(:,1), accel_eval(:,2), pos_eval(:,1), pos_eval(:,2), arc_angle * ones(size(pos_eval,1),1));
        center_to_surf = contact_arc_centered_fcn(radius, accel_eval(:,1), accel_eval(:,2), arc_angle*ones(size(pos_eval,1),1));
        contact_loc = center_to_surf + repmat([0 0 radius],[length(tspan), 1]);
        surface_vel = cross(omega, center_to_surf, 2) + [diff(contact_loc); 0,0,0];% + vel_eval; % Relative to the point in space, so entire translation is already taken into account.

        up_vectors = center_to_surf./sqrt(center_to_surf(:,1).^2 + center_to_surf(:,2).^2 + center_to_surf(:,3).^2);
        
%         up_vector = [0, cos(arc_angle)*radius, sin(arc_angle)*radius];
%         contact_loc = repmat([0 0 radius] + [0, cos(arc_angle)*radius, sin(arc_angle)*radius],[length(tspan), 1]);
%         contact_loc_over_time = contact_loc + pos_eval;
%         surface_vel = cross(omega, contact_loc, 2);

        [jnt_angles_optim, breaks_optim, solinfo_optim, proj_start_pt, result_path_pts, path_rot_integrated] = attempt_kinematics_over_surface( ...
            point_to_project_to_surface, ...
            placement_ang, ...
            tspan, ...
            surface_vel, ...
            up_vectors, ...
            contact_loc_over_time, ...
            geo_data.merged_iiwa(detail_level), ...
            num_pts, ...
            iiwa, ...
            home, ...
            kill_on_bad_sol, ...
            banned);
                if numel(proj_start_pt) == 1
                    error = (length(tspan) - find(all(result_path_pts == 0,2),1,'first') - 1) + 20; % Make it slightly less bad if the integration of the path makes it most of the way.
                    fprintf('Unsuccessful integration of velocity over surface for a cost of: %f\n', error);
                else
                    solres = [solinfo_optim{:}];
                    error = sum([solres.PoseErrorNorm]);%.*(length(solres):-1:1));
                end
%         
%         plot3(proj_start_pt(1), proj_start_pt(2), proj_start_pt(3), '.g', 'MarkerSize', 15, 'Color', cmap_jet(ceil(-log10(error/100)),:));
%         drawnow;
    end
[error, jnt_angles_optim, breaks_optim, solinfo_optim, result_path_pts, path_rot_integrated] = cost_fun(xres(1), xres(2), xres(3), xres(4), 100, false); % Don't kill IK even if bad solution occurs.




campos([1.6970    1.5293    0.9466]);
for i = 1:length(tspan) - 1
    dt = tspan(i+1) - tspan(i);
    rotationQ = 0.5 * quatprod([0, omega(i,:)], rotationQ) * dt + rotationQ;
    rotationQ = rotationQ/norm(rotationQ);
    reverseQ = rotationQ .* [-1 1 1 1]; % MATLAB's reverse right-hand-rule for quaternions. Wow!
    ball_patch.Vertices = quatrotate(reverseQ, ball_verts_untransformed) + repmat(pos_eval(i,:)  + [0, 0, radius], [size(ball_verts_untransformed,1),1]);
    
        % Just for manipulating the blue ghost version.
    surface_to_origin_translation = trvec2tform(-result_path_pts(i,:));
    world_origin_to_goal_pt_translation = trvec2tform(contact_loc_over_time(i,:));
    surf_vel_untransformed_to_planar_vel = rotm2tform(path_rot_integrated(:,:,i)');
    iiwa_tform.Matrix = world_origin_to_goal_pt_translation * surf_vel_untransformed_to_planar_vel * surface_to_origin_translation;
    
    
    
    joint_pos_struct = interpolate_traj(iiwa, breaks_optim, jnt_angles_optim, tspan(i));
    display_at_pose(iiwa, joint_pos_struct);
    
    %     camvec = (angle2dcm(0,0,i/200, 'xyz')*(lin_vel(i,:)*1.2 + [0.0 0 .25])')';
    %     camtarget(pos(i,:));
    %     campos(pos(i,:) + camvec);
    drawnow;
    pause(0.001);
end


end
