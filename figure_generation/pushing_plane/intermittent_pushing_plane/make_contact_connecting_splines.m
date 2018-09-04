function connecting_pps = make_contact_connecting_splines(start_times, end_times, spline_start_pos, spline_end_pos, plane_touch_starts, ...
    plane_touch_ends, contact_vel_start, contact_vel_end, start_accel_x, start_accel_y, end_accel_x, end_accel_y, plane_tilt)

outward_scoop_dist = 0.1;
outward_upward_scoop_dist = 0.05;
above_scoop_dist = 0.2;

%% Make connecting splines.
placeholder_pp = mkpp([0,1],1);
connecting_pps = repmat(placeholder_pp, [size(spline_start_pos,1), 1]);
for i = 1:size(spline_start_pos,1)
    lin_st = spline_start_pos(i,:);
    lin_e = spline_end_pos(i,:);
    lin_st_accel = [start_accel_x(i), start_accel_y(i), 0];
    lin_e_accel = [end_accel_x(i), end_accel_y(i), 0];
    
    plane_lin_start_offset_world_coords = plane_touch_ends(i,1) * isurf_fcn(start_accel_x(i), start_accel_y(i), plane_tilt) + ...
        plane_touch_ends(i,2) * jsurf_fcn(start_accel_x(i), start_accel_y(i), plane_tilt);
    
    plane_lin_end_offset_world_coords = plane_touch_starts(i + 1,1) * isurf_fcn(end_accel_x(i), end_accel_y(i), plane_tilt) + ...
        plane_touch_starts(i + 1,2) * jsurf_fcn(end_accel_x(i), end_accel_y(i), plane_tilt);
    
    lin_st = lin_st + plane_lin_start_offset_world_coords;
    lin_e = lin_e + plane_lin_end_offset_world_coords;
    
    lin_perp = cross(lin_e - lin_st, [0, 0, 1]);
    lin_perp = lin_perp/norm(lin_perp);
    
    way_dir_st = -sign(dot(lin_st_accel, lin_perp));
    way_dir_e =  -sign(dot(lin_e_accel, lin_perp));
    
    if way_dir_st*way_dir_e == 1 % Same sign. We want to scoop outwards around the ball.
        lin_waypt = lin_perp*way_dir_st*outward_scoop_dist + (lin_e + lin_st)/2 + [0, 0, outward_upward_scoop_dist];
    else
        lin_waypt = (lin_e + lin_st)/2 + [0, 0, above_scoop_dist]; % Different sign. Let's go over. We need to switch sides of the ball.
    end
    
    % Position connecting spline.
    lin_waypt_t = (start_times(i) + end_times(i))/2;
    pp_connect = spline([start_times(i), lin_waypt_t, end_times(i)], [contact_vel_start(i,:); lin_st; lin_waypt; lin_e; contact_vel_end(i,:)]');
    connecting_pps(i,1) = pp_connect; 
    
end
end