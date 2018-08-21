function derived_eqns = do_derivations()
%% Dumps derived functions in derived_autogen directory.
syms fax fay faz fn ffx ffy rx ry vx vy ax ay wx wy wz wdx wdy wdz fric_coeff theta real;
syms g m I R positive;
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
% fric_coeff - ground to ball friction coefficient.
% theta - angle from horizontal along ball arc which force is applied

disp('Running derivations.');

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
[ffx_solve, ffy_solve, fax_solve, fay_solve] = solve(F_eqn(1:2), T_eqn(1:2), [ffx, ffy, fax, fay]); % Solve for frictional force and arm xy force components

Fnormal = solve(dot(Fsum, k), fn); % Total normal force.

% Friction cone requirement. NOT YET USED.
friction_required = sqrt(ffx_solve^2 + ffy_solve^2);
friction_max = Fnormal*fric_coeff;

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

equator_contact_velocity = cross([wx,wy,wz], arm_contact_pt_equator) + [vx, vy, 0];

vsurf_general = (vx*i + vy*j) + cross((wx*i + wy*j + wz*k), full_contact_arc); % Surface velocity of ball at contact point, regardless of contact angle along arc.
isurf = simplify(cross(k, -full_contact_arc/norm(full_contact_arc))); % Surface-aligned 'horizontal' component. Note cross( is,js) = vector pointing towards center of ball from contact point.
isurf = simplify(isurf/norm(isurf));
jsurf = simplify(cross(-full_contact_arc/norm(full_contact_arc), isurf)); % Surface-aligned 'vertical component'

v_surfx = dot(vsurf_general, isurf); % This finds the component of velocity
v_surfy = dot(vsurf_general, jsurf);

%% Make functions for various symbolic equations.
write_location = '../derived_autogen/'; % For writing to function files.

derived_eqns.friction_required_fcn = matlabFunction(friction_required, 'File', strcat(write_location, 'friction_required_fcn'));
derived_eqns.x_force_required_fcn = matlabFunction(fax_solve, 'File', strcat(write_location, 'x_force_required_fcn')); % Required x component of force applied by arm.
derived_eqns.y_force_required_fcn = matlabFunction(fay_solve, 'File', strcat(write_location, 'y_force_required_fcn'));
derived_eqns.angular_rate_wx_fcn = matlabFunction(wx, 'File', strcat(write_location, 'angular_rate_wx_fcn'));
derived_eqns.angular_rate_wy_fcn = matlabFunction(wy, 'File', strcat(write_location, 'angular_rate_wy_fcn'));
derived_eqns.contact_arc_fcn = matlabFunction(full_contact_arc_shifted', 'File', strcat(write_location, 'contact_arc_fcn'));
derived_eqns.contact_arc_centered_fcn = matlabFunction(full_contact_arc', 'File', strcat(write_location, 'contact_arc_centered_fcn'));
derived_eqns.equator_contact_velocity_fcn = matlabFunction(equator_contact_velocity, 'File', strcat(write_location, 'equator_contact_velocity_fcn'));
derived_eqns.world_contact_velocity_fcn = matlabFunction(vsurf_general, 'File', strcat(write_location, 'world_contact_velocity_fcn'));
derived_eqns.v_surfx_fcn = matlabFunction(v_surfx, 'File', strcat(write_location, 'v_surfx_fcn'));
derived_eqns.v_surfy_fcn = matlabFunction(v_surfy, 'File', strcat(write_location, 'v_surfy_fcn'));
derived_eqns.isurf_fcn = matlabFunction(isurf', 'File', strcat(write_location, 'isurf_fcn'));
derived_eqns.jsurf_fcn = matlabFunction(jsurf', 'File', strcat(write_location, 'jsurf_fcn'));
end
