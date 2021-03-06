function spline_optim = qp_spline(breaks, knots, pinned_spacing, endpt_condition)
% SPLINE_OPTIM = QP_SPLINE(BREAKS, KNOTS, PINNED_SPACING) Optimize a cubic
% spline's coefficients with added segments.
%   Takes breaks and knots which the resulting spline must obey. Adds
%   aditional segments based on pinned_spacing. 1 means no extras.
%   Optimizes the added
%   segment's coefficients to minimize the integral of acceleration
%   squared.

validateattributes(breaks, {'single', 'double'}, {'real', 'vector', 'increasing'});
validateattributes(knots, {'single', 'double'}, {'real'});
validateattributes(pinned_spacing, {'numeric'}, {'scalar', 'integer'});
validateattributes(endpt_condition, {'string', 'char'}, {});

if size(knots,1) ~= length(breaks)
    if size(knots,2) == length(breaks)
        knots = knots';
    else
        error('Number of knots should equal number of breaks.');
    end
end

pts = pinned_spacing*size(knots,1) - pinned_spacing + 1;
segs = pts - 1;

syms m x t_p;
a = sym('a', [segs, 4]); % Trajectory polynomial coefficients. Cubic.
b = sym('b', [segs, 4]); % Trajectory polynomial coefficients. Cubic.

% We add extra "subknots" in between the supplied ones. This requires
% additional "subbreaks".
extended_breaks = interp1(1:length(breaks), breaks, linspace(1,length(breaks), pts))';
tst = extended_breaks(1:end-1,1); % Start of each segment
te = extended_breaks(2:end,1); % End time of each segment.

poly_eval_fcn = @(t, coeff)(coeff(:,1) .* t.^3 + coeff(:,2) .* t.^2 + coeff(:,3) .* t + coeff(:,4));
poly_dt_fcn = @(t, coeff)(3 * coeff(:,1) .* t.^2 + 2 * coeff(:,2) .* t + coeff(:,3));
poly_ddt_fcn = @(t, coeff)(6 * coeff(:,1) .* t + 2 * coeff(:,2));

xst = poly_eval_fcn(0, a);
xe = poly_eval_fcn(te - tst, a);
vxst = poly_dt_fcn(0, a);
vxe = poly_dt_fcn(te - tst, a);
axst = poly_ddt_fcn(0, a);
axe = poly_ddt_fcn(te - tst, a);

yst = poly_eval_fcn(0, b);
ye = poly_eval_fcn(te - tst, b);
vyst = poly_dt_fcn(0, b);
vye = poly_dt_fcn(te - tst, b);
ayst = poly_ddt_fcn(0, b);
aye = poly_ddt_fcn(te - tst, b);

accel_generic_x = poly_ddt_fcn(t_p, a);
accel_generic_y = poly_ddt_fcn(t_p, b);
cost_integral = 0;
for i = 1:size(tst,1)
    cost_integral = cost_integral + int(accel_generic_x(i)^2, t_p, 0 ,te(i) - tst(i));
    cost_integral = cost_integral + int(accel_generic_y(i)^2, t_p, 0 ,te(i) - tst(i));
end

pinned_pts_x = [xst(1:pinned_spacing:end); xe(end)] == knots(:,1);
pinned_pts_y = [yst(1:pinned_spacing:end); ye(end)] == knots(:,2);

x_constraints = [
    pinned_pts_x;
    xe(1:end - 1) == xst(2:end);
    vxe(1:end - 1) == vxst(2:end);
    axe(1:end - 1) == axst(2:end)];

y_constraints = [
    pinned_pts_y;
    ye(1:end - 1) == yst(2:end);
    vye(1:end - 1) == vyst(2:end);
    aye(1:end - 1) == ayst(2:end)];

% How to handle end point conditions.
switch endpt_condition   
    case 'periodic' % Accelerations and velocities.
        x_constraints = [x_constraints;
            vxe(end) == vxst(1);
            axe(end) == axst(1)];
        y_constraints = [y_constraints;
            vye(end) == vyst(1);
            aye(end) == ayst(1)];
    case 'velocities' % Just velocities.
        x_constraints = [x_constraints;
            vxe(end) == vxst(1)];
        y_constraints = [y_constraints;
            vye(end) == vyst(1)];
    case 'zero' % At rest
        x_constraints = [x_constraints;
            vxe(end) == 0;
            vxst(1) == 0;
            axe(end) == 0;
            axst(1) == 0];
        y_constraints = [y_constraints;
            vye(end) == 0;
            vyst(1) == 0;         
            aye(end) == 0;
            ayst(1) == 0];
    case 'free' % Only position constraints.
    otherwise
        warning('Unrecognized endpoint condition.');
        x_constraints = [x_constraints;
            vxe(end) == 0;
            vxst(1) == 0];
        y_constraints = [y_constraints;
            vye(end) == 0;
            vyst(1) == 0];
end

x_coeffs = reshape(a.',numel(a),1);
y_coeffs = reshape(b.',numel(b),1);

all_constraints = [x_constraints; y_constraints];
all_coeffs = [x_coeffs; y_coeffs];

[A_cost, B_cost] = equationsToMatrix(jacobian(cost_integral, all_coeffs), all_coeffs);
A_cost_num = eval(A_cost)';

[A,B] = equationsToMatrix(all_constraints, all_coeffs);
A_num = eval(A);
B_num = eval(B);

opt = optimset('quadprog');
opt.Display = 'off';
opt.OptimalityTolerance = 1e-12;

[sol, fval] = quadprog(A_cost_num, zeros(size(all_coeffs)), [], [], A_num, B_num, [], [], [], opt);
% x_sol = sol(1:length(sol)/2);
% y_sol = sol(length(sol)/2 + 1:end);

coeff_sol = reshape(sol,[4, length(sol)/4]).';
ppx = mkpp([tst;te(end)], coeff_sol(1:length(coeff_sol)/2,:));
ppy = mkpp([tst;te(end)], coeff_sol(length(coeff_sol)/2 + 1:end,:));

spline_optim = spline_concat_in_dimension(ppx ,ppy); % Combine the separate ppx and ppy piecewise polys into one.
end
