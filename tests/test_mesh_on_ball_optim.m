function test_mesh_on_ball_optim
close all; clear all;
addpath ../;
addpath ../vis/;
addpath ../geometry/;
addpath ../derived_autogen/;
addpath ../data/;
addpath ../path_optim/;


%% Import data for the dummy planning foot and the banned regions on it.
dummy_foot_dat = get_mesh_data('dummy_manipulator_mid_res');
banned_region_dat = get_mesh_data('manipulator_banned1');

% Not the main scene, just shows where samples are being pulled from on the
% dummy foot.
sampling_figure = figure(103);
hold on;
sampling_foot_patch = patch('Faces', dummy_foot_dat.faces, 'Vertices', dummy_foot_dat.vertices, 'FaceNormals', dummy_foot_dat.face_normals, ...
    'VertexNormals', dummy_foot_dat.vertex_normals);
sampling_foot_patch.EdgeColor = 'none';
sampling_foot_patch.FaceColor = 'flat';
sampling_foot_patch.FaceVertexCData = repmat([0 0 1], [size(sampling_foot_patch.Faces,1),1]);
sampling_foot_patch.FaceAlpha = 0.8;
sampling_foot_patch.BackFaceLighting = 'lit';

banned_region_patch = patch('Faces', banned_region_dat.faces, 'Vertices', banned_region_dat.vertices, 'FaceNormals', banned_region_dat.face_normals, ...
    'VertexNormals', banned_region_dat.vertex_normals);
banned_region_patch.EdgeColor = 'none';
banned_region_patch.FaceAlpha = 0.3;
banned_region_patch.BackFaceLighting = 'lit';

%% Make cylinder around the dummy link and pick permissible areas to sample from.
maxx = max(dummy_foot_dat.vertices(:,1),[],1);
minx = min(dummy_foot_dat.vertices(:,1),[],1);
maxy = max(dummy_foot_dat.vertices(:,2),[],1);
miny = min(dummy_foot_dat.vertices(:,2),[],1);
maxz = max(dummy_foot_dat.vertices(:,3),[],1);
minz = min(dummy_foot_dat.vertices(:,3),[],1);

% Longitudinal axis aligned with x on the dummy link.
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
min_ang = 0; % Min and max angle in cylindrical coordinates that we're allowed to sample between.
max_ang = 2*pi;
cyl_h_ext = 0.05;
low_line = plot3([cyl_off_x - cyl_h_ext, cyl_off_x + cyl_height + cyl_h_ext], [cyl_rad*cos(min_ang), cyl_rad*cos(min_ang)] + cyl_off_y, [cyl_rad*sin(min_ang), cyl_rad*sin(min_ang)] + cyl_off_z, '-g', 'LineWidth', 2);
high_line = plot3([cyl_off_x - cyl_h_ext, cyl_off_x + cyl_height + cyl_h_ext], [cyl_rad*cos(max_ang), cyl_rad*cos(max_ang)] + cyl_off_y, [cyl_rad*sin(max_ang), cyl_rad*sin(max_ang)] + cyl_off_z, '-r', 'LineWidth', 2);

xlabel('x');
ylabel('y');
zlabel('z');
view(3);
daspect([1,1,1]);
camlight HEADLIGHT;
hold off;

%% Set up world scene.

% World scene.
scene_fig = make_visualizer_scene();

% Ball.
hold on;
ball_radius = 0.1;
[ball_patch, ball_verts_untransformed] = make_ball(ball_radius);

% Whole robot.
iiwa = IIWAImporter(scene_fig);
link_name = 'iiwa_link_6'; % Tip of ee. iiwa_link_ee is just the last full link.
home = iiwa.home_config;
guess = home;

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
total_time = 5;
num_evaluation_pts = 50;
pos_pp = get_path_pp('small_arc_knot', total_time);

[tspan, posspan, velspan, accelspan, omegaspan, quatspan, ...
    world_contact_loc_desired_fcn, contact_loc_desired_rel_com_fcn] = evaluate_spline(pos_pp, ball_radius, num_evaluation_pts);

draw_path_and_accel(posspan, accelspan, 3); % Draw out the path and acceleration arrows.

%% Set up optimization
just_playback = true;
figure(sampling_figure); % Bring back the figure for showing sampling.
cmap_jet = jet(13); % Colormap to show how good each point is. Hotter == better.
visualize_sampling = false;
hold on;
cmaes_opts = cmaes;

% Bounds: cylinder Z, cylinder angle, approach angle, arc angle.
cmaes_opts.LBounds = [-cyl_h_ext, min_ang, 0, 0]';
cmaes_opts.UBounds = [cyl_height + cyl_h_ext, max_ang, 2*pi, pi/2-0.03]';

% Stopping params + important stuff
cmaes_opts.StopFitness = 5e-8;
cmaes_opts.MaxFunEvals = 300;
cmaes_opts.PopSize = 8;

% Relatively stupid settings.
cmaes_opts.LogPlot = 'off';
cmaes_opts.LogModulo = 0;
cmaes_opts.ReadSignals = 'off';
cmaes_opts.SaveVariables = 'off';
cmaes_opts.DispModulo = 25;

% Initial guess.
xres = [    0.1268
    0.3809
    6.1150
    1.2588];

%% Run the optimization.
if ~just_playback
    [xres, fval] = cmaes(@cost_fun_wrap, xres, 0.2*(cmaes_opts.UBounds - cmaes_opts.LBounds), cmaes_opts)
end
% Wrap the cost function in 'SISO' form for CMAES to use.
    function err = cost_fun_wrap(X)
        err = cost_fun(X(1,:), X(2,:), X(3,:),X(4,:), 10, true);
    end

% Cost function with full result outputs.
    function [error, jnt_angles_optim, breaks_optim, solinfo_optim, result_path_pts, path_rot_integrated, world_contact_desired_span, up_vector_span] = ...
            cost_fun(proj_height, proj_ang, placement_ang, arc_angle, num_pts, kill_on_bad_sol)
        
        point_to_project_to_surface = [proj_height + cyl_off_x, cyl_rad*cos(proj_ang) + cyl_off_y, cyl_rad*sin(proj_ang) + cyl_off_z];
        
        % Evaluate terms which depend on the chosen arc angle.
        world_contact_desired_span = world_contact_loc_desired_fcn(arc_angle*ones(size(tspan)));
        contact_desired_rel_com_span = contact_loc_desired_rel_com_fcn(arc_angle*ones(size(tspan)));
        
        up_vector_span = contact_desired_rel_com_span/ball_radius;
        surface_vel_span = cross(omegaspan, contact_desired_rel_com_span, 2); % Do we want a diff(contact_loc) term?
        
        % Attempt the kinematics.
        [jnt_angles_optim, breaks_optim, solinfo_optim, proj_start_pt, result_path_pts, path_rot_integrated, fail_flag] = attempt_kinematics_over_surface( ...
            point_to_project_to_surface, ...
            placement_ang, ...
            tspan, ...
            surface_vel_span, ...
            up_vector_span, ...
            world_contact_desired_span, ...
            dummy_foot_dat, ...
            num_pts, ...
            iiwa, ...
            home, ...
            kill_on_bad_sol, ...
            banned_region_dat);
        
        % Catch velocity integration over surface.
        if fail_flag
            error = (length(tspan) - find(all(result_path_pts == 0,2),1,'first') - 1) + 20; % Make it slightly less bad if the integration of the path makes it most of the way.
            fprintf('Unsuccessful integration of velocity over surface for a cost of: %f\n', error);
        else
            solres = [solinfo_optim{:}];
            error = sum([solres.PoseErrorNorm]);%.*(length(solres):-1:1));
        end
        
        if visualize_sampling
            plot3(proj_start_pt(1), proj_start_pt(2), proj_start_pt(3), '.g', 'MarkerSize', 15, 'Color', cmap_jet(ceil(-log10(error/1000)),:));
            drawnow;
        end
    end

%% Evaluate the optimization winner.
[error, jnt_angles_optim, breaks_optim, solinfo_optim, result_path_pts, path_rot_integrated, world_contact_desired_span, up_vector_span] = cost_fun(xres(1), xres(2), xres(3), xres(4), 25, false); % Don't kill IK even if bad solution occurs.



%% Play the winning solution.
campos([1.6970    1.5293    0.9466]);
for i = 1:length(tspan) - 1
    
    ball_patch.Vertices = quatrotate(quatspan(i,:), ball_verts_untransformed) + repmat(posspan(i,:)  + [0, 0, ball_radius], [size(ball_verts_untransformed,1),1]);
    
    % Just for manipulating the blue ghost version.
    surface_to_origin_translation = trvec2tform(-result_path_pts(i,:));
    world_origin_to_goal_pt_translation = trvec2tform(world_contact_desired_span(i,:));
    
    surf_vel_untransformed_to_planar_vel = rotm2tform(path_rot_integrated(:,:,i)');
    ball_contact_shift_tform = rotm2tform(get_rotation_from_vecs(up_vector_span(1,:), up_vector_span(i,:))); % upvec_rot(:,:,i));%
    dummy_foot_tform.Matrix = world_origin_to_goal_pt_translation * ball_contact_shift_tform * surf_vel_untransformed_to_planar_vel * surface_to_origin_translation;
    
    joint_pos_struct = interpolate_traj(iiwa, breaks_optim, jnt_angles_optim, tspan(i));
    display_at_pose(iiwa, joint_pos_struct);
    
    drawnow;
    pause(0.001);
end


end
