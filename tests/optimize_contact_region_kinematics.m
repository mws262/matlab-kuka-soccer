function [solution, jnt_angles_optim, breaks_optim, result_path_pts, path_rot_integrated, world_contact_desired_span, up_vector_span] = ...
    optimize_contact_region_kinematics(robot, initial_guess, pos_pp, num_eval_pts, ball_radius, plot_flag, just_evaluate_guess)

%% Calculate stuff based on the position spline.
[tspan, posspan, velspan, accelspan, omegaspan, quatspan, ...
    world_contact_loc_desired_fcn, contact_loc_desired_rel_com_fcn] = evaluate_spline(pos_pp, ball_radius, num_eval_pts);

home = robot.home_config;

%% Import data for the dummy planning foot and the banned regions on it.
dummy_foot_dat = get_mesh_data('dummy_manipulator_mid_res');
banned_region_dat = get_mesh_data('manipulator_banned1');

% Not the main scene, just shows where samples are being pulled from on the
% dummy foot.
if plot_flag
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
end

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
min_ang = 0; % Min and max angle in cylindrical coordinates that we're allowed to sample between.
max_ang = 2*pi;
cyl_h_ext = 0.05;

if plot_flag
    [y_cyl, z_cyl, x_cyl] = cylinder(cyl_rad, 20);
    cyl_patch = patch(surf2patch(x_cyl, y_cyl, z_cyl, 'triangles'))
    cyl_patch.Vertices(:,1) = cyl_patch.Vertices(:,1)*cyl_height + cyl_off_x;
    cyl_patch.Vertices(:,2) = cyl_patch.Vertices(:,2) + cyl_off_y;
    cyl_patch.Vertices(:,3) = cyl_patch.Vertices(:,3) + cyl_off_z;
    cyl_patch.FaceAlpha = 0.3;
    low_line = plot3([cyl_off_x - cyl_h_ext, cyl_off_x + cyl_height + cyl_h_ext], [cyl_rad*cos(min_ang), cyl_rad*cos(min_ang)] + cyl_off_y, [cyl_rad*sin(min_ang), cyl_rad*sin(min_ang)] + cyl_off_z, '-g', 'LineWidth', 2);
    high_line = plot3([cyl_off_x - cyl_h_ext, cyl_off_x + cyl_height + cyl_h_ext], [cyl_rad*cos(max_ang), cyl_rad*cos(max_ang)] + cyl_off_y, [cyl_rad*sin(max_ang), cyl_rad*sin(max_ang)] + cyl_off_z, '-r', 'LineWidth', 2);
    
    xlabel('x');
    ylabel('y');
    zlabel('z');
    view(3);
    daspect([1,1,1]);
    camlight HEADLIGHT;
    cmap_jet = jet(13); % Colormap to show how good each point is. Hotter == better.
    hold on;
end

%% Set up optimization
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

if ~just_evaluate_guess
    [solution, cost_val] = cmaes(@cost_fun_wrap, initial_guess, 0.2*(cmaes_opts.UBounds - cmaes_opts.LBounds), cmaes_opts)
else
    solution = initial_guess;
end
%% Evaluate the optimization winner.
[error, jnt_angles_optim, breaks_optim, solinfo_optim, result_path_pts, path_rot_integrated, world_contact_desired_span, up_vector_span] = ...
    cost_fun(solution(1), solution(2), solution(3), solution(4), 25, false); % Don't kill IK even if bad solution occurs.


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
            robot, ...
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
        
        if plot_flag
            plot3(proj_start_pt(1), proj_start_pt(2), proj_start_pt(3), '.g', 'MarkerSize', 15, 'Color', cmap_jet(ceil(-log10(error/1000)),:));
            drawnow;
        end
    end
end

