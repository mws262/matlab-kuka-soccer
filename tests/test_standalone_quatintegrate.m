clear all; close all;
addpath ../derived_autogen/;
addpath ../geometry/;
% Changing the as-we-go euler integration of quaternions to ahead-of-time
% rk4.
%
% Still feels like a dumb thing to do, especially since we have a full
% expression for omega. Still, it's better than euler from before. If the
% normalization is taken away from both rk4 and euler, euler loses unit
% length by about a factor of bajillion more.

%% Make some generic ball path stuff + states
% Arc
tend = 5;
R = 0.1;
offset = [0.4;0;0];
knots = [0,-R*1.5,0; R/2,0,0; 0, R*1.5,0]' + offset;

breaks = linspace(0, tend, size(knots,2));
pos_pp = spline(breaks, [[0;0;0], knots, [0;0;0]]);

ball_radius = 0.1;
vel_pp = fnder(pos_pp);
accel_pp = fnder(vel_pp);
points = 125;
tspan = linspace(0, tend, points)';
% Replicate everything if we want to do multiple laps.
pos_eval = ppval(pos_pp, tspan)';
vel_eval = ppval(vel_pp, tspan)';
accel_eval = ppval(accel_pp, tspan)';

% States, etc
zero_vec_pts = zeros(size(tspan));
omega = [angular_rate_wx_fcn(ball_radius, vel_eval(:,2)), angular_rate_wy_fcn(ball_radius, vel_eval(:,1)), zero_vec_pts];

rotationQ = [1 0 0 0]; % Quaternion representing ball rotation.


quatspan_rk4 = quatintegrate(tspan, rotationQ, omega);

quatspan_euler = zeros(length(tspan),4);
quatspan_euler(1,:) = rotationQ;
for i = 1:length(tspan) - 1
    dt = tspan(i+1) - tspan(i);
    quatspan_euler(i + 1,:) = 0.5 * quatmultiply([0, omega(i,:)], quatspan_euler(i,:)) * dt + quatspan_euler(i,:);
    quatspan_euler(i + 1,:) = quatspan_euler(i + 1,:)/norm(quatspan_euler(i + 1,:));
end
quatspan_euler(:,1) = -quatspan_euler(:,1);

plot(quatspan_rk4);
figure;
plot(quatspan_euler);
