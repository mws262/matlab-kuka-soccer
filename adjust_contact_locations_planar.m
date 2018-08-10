function [touch_section_starts, touch_section_ends, min_plane_x_dim, min_plane_y_dim] = adjust_contact_locations_planar(tspan, surface_vel_x, surface_vel_y, linear_start_times, linear_end_times)
%% Exploring locations on manipulator to touch.

% surfx_integral_eval = cumtrapz(tspan, surface_vel_x); % Evaluate the x coordinate integral of surface velocity, reflecting distance moved along the manipulator.
% surfy_integral_eval = cumtrapz(tspan, surface_vel_y);

% surfx_linear_start = interp1(tspan, surfx_integral_eval, accel_zero_break_start); % X and y position on the push-plane for each
% surfx_linear_end = interp1(tspan, surfx_integral_eval, accel_zero_break_end);
% surfy_linear_start = interp1(tspan, surfy_integral_eval, accel_zero_break_start);
% surfy_linear_end = interp1(tspan, surfy_integral_eval, accel_zero_break_end);

debug_plot = false;

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

if debug_plot
    figure(202);
    hold on;
end
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
    if debug_plot
        plot(surfx_integral_eval_active_shift, surfy_integral_eval_active_shift, surfx_integral_eval_active_shift(1), surfy_integral_eval_active_shift(1),'g.', surfx_integral_eval_active_shift(end), surfy_integral_eval_active_shift(end), 'r.', 'MarkerSize', 20);
    end
end

if debug_plot
    hold off;
end
% actual_pl = plot(0,0,'.','MarkerSize', 10);
end