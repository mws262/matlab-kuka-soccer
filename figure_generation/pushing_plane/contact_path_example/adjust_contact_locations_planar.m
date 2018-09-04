function [touch_section_starts, touch_section_ends, min_plane_x_dim, min_plane_y_dim] = adjust_contact_locations_planar(tspan, surface_vel_x, surface_vel_y, linear_start_times, linear_end_times)
% Picks contact points *only* for the planar case.
%% Exploring locations on manipulator to touch.

% surfx_integral_eval = cumtrapz(tspan, surface_vel_x); % Evaluate the x coordinate integral of surface velocity, reflecting distance moved along the manipulator.
% surfy_integral_eval = cumtrapz(tspan, surface_vel_y);

% surfx_linear_start = interp1(tspan, surfx_integral_eval, accel_zero_break_start); % X and y position on the push-plane for each
% surfx_linear_end = interp1(tspan, surfx_integral_eval, accel_zero_break_end);
% surfy_linear_start = interp1(tspan, surfy_integral_eval, accel_zero_break_start);
% surfy_linear_end = interp1(tspan, surfy_integral_eval, accel_zero_break_end);

debug_plot = true;

nonlin_starts = [0, linear_end_times];
nonlin_ends = [linear_start_times, tspan(end)];

if debug_plot
    figure(201);
    hold on;
end

max_range_x = 0;
max_range_y = 0;

%% Before rearranging.
for i = 1:length(nonlin_starts)
    % Integrate each path segment.
    active_idx = find(nonlin_starts(i) >= tspan, 1, 'last'):find(nonlin_ends(i) >= tspan, 1, 'last');
    surfx_integral_eval_active = cumtrapz(tspan(active_idx), surface_vel_x(active_idx));
    surfy_integral_eval_active = cumtrapz(tspan(active_idx), surface_vel_y(active_idx));
    range_x = range(surfx_integral_eval_active);
    range_y = range(surfy_integral_eval_active);
    if range_x > max_range_x % Keep track of the longest path in each dimension.
        max_range_x = range_x;
    end
    if range_y > max_range_y
        max_range_y = range_y;
    end
    if debug_plot
        plot(surfx_integral_eval_active, surfy_integral_eval_active, surfx_integral_eval_active(1), surfy_integral_eval_active(1),'g.', surfx_integral_eval_active(end), surfy_integral_eval_active(end), 'r.', 'MarkerSize', 20);
    end
end
min_plane_x_dim = max_range_x;
min_plane_y_dim = max_range_y;

if debug_plot
    hold off;
end

fig = figure(202);
hold on;
% Plane representing an object pushing the ball.
[plane_x, plane_y] = meshgrid(-0.5:0.1:0.5); % Generate x and y data
[plane_z] = zeros(size(plane_x, 1)); % Generate z data
plane_patch = patch(surf2patch(plane_x, plane_y, plane_z)); % Makes the plane, but the normal vector is in the z-direction.
plane_patch.FaceColor = [0.5,0.5,0.8];
plane_patch.EdgeColor = [0, 0, 1];
plane_patch.LineWidth = 1;
plane_patch.EdgeAlpha = 1;
plane_patch.FaceAlpha = 0.2;


%% Subtract offsets from each.
touch_section_starts = zeros(length(nonlin_starts),2);
touch_section_ends = zeros(length(nonlin_starts),2);
for i = 1:length(nonlin_starts)
    active_idx = find(nonlin_starts(i) >= tspan, 1, 'last'):find(nonlin_ends(i) >= tspan, 1, 'last');
    surfx_integral_eval_active_shift = cumtrapz(tspan(active_idx), surface_vel_x(active_idx)); % TODO: fix duplicate computation. It's already fast so low priority.
    surfy_integral_eval_active_shift = cumtrapz(tspan(active_idx), surface_vel_y(active_idx));
    range_x = range(surfx_integral_eval_active_shift);
    range_y = range(surfy_integral_eval_active_shift);
    min_x = min(surfx_integral_eval_active_shift);
    min_y = min(surfy_integral_eval_active_shift);
    
    surfx_integral_eval_active_shift = surfx_integral_eval_active_shift - min_x - range_x/2;
    surfy_integral_eval_active_shift = surfy_integral_eval_active_shift - min_y - range_y/2;
    
    touch_section_starts(i, :) = [surfx_integral_eval_active_shift(1), surfy_integral_eval_active_shift(1)];
    touch_section_ends(i, :) = [surfx_integral_eval_active_shift(end), surfy_integral_eval_active_shift(end)];
    if debug_plot  && i ==3% Just plot one
        velx = surface_vel_x(active_idx);
        vely = surface_vel_y(active_idx);
        ts = tspan(active_idx);
        save('surface_vel_dat', 'velx', 'vely', 'ts');
        
        surf_path_pl = plot(surfx_integral_eval_active_shift, surfy_integral_eval_active_shift)
        surf_path_pl.Color = [0,0,0];
        surf_path_pl.LineWidth = 5;
        plot(surfx_integral_eval_active_shift(1), surfy_integral_eval_active_shift(1),'g.', 'MarkerSize', 50) 
        plot(surfx_integral_eval_active_shift(end), surfy_integral_eval_active_shift(end), 'r.', 'MarkerSize', 50);
    end
end




fig.Position = [0, 0, 1200, 900];
plane_patch.Vertices(:,1) = plane_patch.Vertices(:,1)*min_plane_x_dim; % Rotate the plane to align with the x axis (i.e. normal in y-direction).
plane_patch.Vertices(:,2) = plane_patch.Vertices(:,2)*min_plane_y_dim;
axis equal;
fig.Color = [1,1,1];
fig.Children.Visible = 'off';
campos([0.0,0.1, -0.2]);
camva(70);
save2pdf('../../../data/images/pushing_plane_contact_path.pdf', fig, 600);
hold off;
% actual_pl = plot(0,0,'.','MarkerSize', 10);
end