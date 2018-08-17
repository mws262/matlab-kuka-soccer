function [tspan, posspan, velspan, accelspan, omegaspan, quatspan, ...
    world_contact_loc_desired_fcn, contact_loc_desired_rel_com_fcn] = evaluate_spline(pos_pp, ball_radius, num_pts)
%PROCESS_SPLINE Given a spline path, evaluate at time steps. 
% Takes:
% pos_pp -- piecewise polynomial representing the desired COM path on the
% ground.
% ball_radius -- Radius of the manipulated ball.
% num_pts -- number of points to divide the time interval into and evaluate
% over.
%
% Returns:
% tspan -- time from first to last break in the given pp, divided equally
% into num_pts.
% posspan -- ball ground contact loc over time.
% velspan -- ball COM velocities over time.
% accelspan -- ball COM accelerations over time.
% omegaspan -- ball world angular rates over time.
% quatspan -- ball world orientations over time as quaternions.
% world_contact_loc_desired_fcn -- desired contact location on the ball
% (i.e. opposite acceleration vec) relative to the world origin, 
% parameterized still by angle along the arc.
% contact_loc_desired_rel_com_fcn -- desired contact location on the ball
% relative to the COM of the ball, in the world frame of reference. Still
% parameterized by angle along the contact arc.
%  

tspan = linspace(pos_pp.breaks(1), pos_pp.breaks(end), num_pts)';

% Take derivatives to get piecewise polynomials for velocity and
% acceleration.
vel_pp = fnder(pos_pp);
accel_pp = fnder(vel_pp);

% Evaluate numerically.
posspan = ppval(pos_pp, tspan)';
velspan = ppval(vel_pp, tspan)';
accelspan = ppval(accel_pp, tspan)';

% Find derived quantities.
zero_vec_pts = zeros(size(tspan));
omegaspan = [angular_rate_wx_fcn(ball_radius, velspan(:,2)), angular_rate_wy_fcn(ball_radius, velspan(:,1)), zero_vec_pts]; % Angular rate.

quat_init = [1 0 0 0]; % Quaternion representing ball rotation. For now, there's no reason to have any other initial position.
quatspan = quatintegrate(tspan, quat_init, omegaspan); % Orientation

% Return some useful function handles, with already known fields filled in.
world_contact_loc_desired_fcn = @(arc_angles_over_time)(contact_arc_fcn(ball_radius, accelspan(:,1), accelspan(:,2), posspan(:,1), posspan(:,2), arc_angles_over_time));
contact_loc_desired_rel_com_fcn = @(arc_angles_over_time)(contact_arc_centered_fcn(ball_radius, accelspan(:,1), accelspan(:,2), arc_angles_over_time));

end

