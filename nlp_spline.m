function [ppx, ppy] = nlp_spline(breaks, knots, pinned_spacing, adjacent_segment_product_scaling, varargin)
% This tries to optimize the shape of spline to minimize sections which are
% NOT linear. This is done by optimizing the coefficients of cubic
% polynomials representing the spline. All polynomials go from time 0 to
% some fixed time step. the polynomials must match position and velocities
% at their ends. Granularity is changed by altering pinned spacing.
% "Squareness" is tuned with adjacent_segment_product_scaling (0-1) (higher ==
% more). "Squareness" is achieved by having a cost associated with the
% PRODUCT of adjacent segment's integrals of acceleration.
periodic = true;

%% Formulate the problem symbolically TODO: well... don't do that.
pts = pinned_spacing*size(knots,1) - pinned_spacing + 1;
segs = pts - 1;

syms t_p;
a = sym('a', [segs, 4]); % Trajectory polynomial coefficients. Cubic. For X coordinate
b = sym('b', [segs, 4]); % Trajectory polynomial coefficients. Cubic. For Y coordinate

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

% Cost is integrated over all segments.
accel_generic_x = poly_ddt_fcn(t_p, a);
accel_generic_y = poly_ddt_fcn(t_p, b);
cost_integral = 0;
for i = 2:size(tst,1) - 1
    squareness_term = int(accel_generic_x(i).^2 + accel_generic_y(i).^2, t_p, 0 ,te(i) - tst(i))*...
        int(accel_generic_x(i+1).^2 + accel_generic_y(i+1).^2, t_p, 0 ,te(i+1) - tst(i+1))*...
        int(accel_generic_x(i-1).^2 + accel_generic_y(i-1).^2, t_p, 0 ,te(i-1) - tst(i-1));
    
    %    effort_term = int(accel_generic_x(i).^2, t_p, 0 ,te(i) - tst(i)) + int(accel_generic_y(i).^2, t_p, 0 ,te(i) - tst(i));
    
    cost_integral = cost_integral + adjacent_segment_product_scaling*squareness_term;
end

% Enforce that some points MUST pass through pinned positions.
pinned_pts_x = [xst(1:pinned_spacing:end); xe(end)] == knots(:,1);
pinned_pts_y = [yst(1:pinned_spacing:end); ye(end)] == knots(:,2);

x_constraints = [
    pinned_pts_x;
    xe(1:end - 1) == xst(2:end); %Adjacent segments must align
    vxe(1:end - 1) == vxst(2:end); %Adjacent velocities must align.
    axe(1:end - 1) == axst(2:end)
    ];
y_constraints = [
    pinned_pts_y;
    ye(1:end - 1) == yst(2:end);
    vye(1:end - 1) == vyst(2:end);
    aye(1:end - 1) == ayst(2:end)
    ];

if periodic
    x_constraints = [x_constraints; vxe(end) == vxst(1)]; % Initial velocity matches final velocity (periodic condition).
    y_constraints = [y_constraints; vye(end) == vyst(1)];
else
    x_constraints = [x_constraints; vxe(end) == 0; vxst(1) == 0]; % Initial velocity matches final velocity (periodic condition).
    y_constraints = [y_constraints; vye(end) == 0; vyst(1) == 0];
end

x_coeffs = reshape(a.',numel(a),1);
y_coeffs = reshape(b.',numel(b),1);

all_constraints = [x_constraints; y_constraints];
all_coeffs = [x_coeffs; y_coeffs];

effort_cost = 10*sum(all_coeffs(1:4:end).^2 + all_coeffs(2:4:end).^2);
cost_fun = matlabFunction(cost_integral + (1 - adjacent_segment_product_scaling)*effort_cost, 'Vars', {all_coeffs});

[A,B] = equationsToMatrix(all_constraints, all_coeffs);
A_num = eval(A);
B_num = eval(B);

opt = optimset('fmincon');
opt.Algorithm = 'sqp';
opt.Display = 'iter';
opt.MaxFunctionEvaluations = 10000;

% We can be provided with other piecewise polynomials from which to pull a
% guess.
if size(varargin,2) == 2
    disp('NLP spline received a guess pp');
    ppx_guess = varargin{1};
    ppy_guess = varargin{2};
    if ppx_guess.pieces ~= segs
        error('provided polynomial guess has a different number of segments');
    end
    
    ppx_guess_coefs = ppx_guess.coefs';
    ppy_guess_coefs = ppy_guess.coefs';
    
    x0 = [ppx_guess_coefs(:); ppy_guess_coefs(:)];
else
    x0 = rand(size(all_coeffs)) - 0.5;
end


[sol, fval] = fmincon(cost_fun, x0, [],[], A_num, B_num,[],[],[],opt); %quadprog(A_cost_num, zeros(size(all_coeffs)), [], [], A_num, B_num, [], [], [], opt);

coeff_sol = reshape(sol,[4, length(sol)/4]).';
ppx = mkpp([tst;te(end)], coeff_sol(1:length(coeff_sol)/2,:));
ppy = mkpp([tst;te(end)], coeff_sol(length(coeff_sol)/2 + 1:end,:));
end
