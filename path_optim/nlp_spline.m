function spline_optim = nlp_spline(user_problem, user_options)
% This tries to optimize the shape of spline to minimize sections which are
% NOT linear. This is done by optimizing the coefficients of cubic
% polynomials representing the spline. All polynomials go from time 0 to
% some fixed time step. the polynomials must match position and velocities
% at their ends. Granularity is changed by altering pinned spacing.
% "Squareness" is tuned with adjacent_segment_product_scaling (0-1) (higher ==
% more). "Squareness" is achieved by having a cost associated with the
% PRODUCT of adjacent segment's integrals of acceleration.

%% Problem structure.
default_problem.pinned_breaks = []; % Breaks which are mandatory for the path.
default_problem.pinned_knots = []; % Knot points which the final solution must pass through.
default_problem.polys_per_pinned_knot = 3; % Number of polynomials to put between each mandatory knot.

default_problem.adjacent_segment_product_scaling = 1; % Sort of smoothing term. 1 means more emphasis on linear section. 0 is more about smoothness.
default_problem.guess_pp = ''; % If guess pp is provided, it will use it as a guess. If not optim_initial_guess will be used.
default_problem.optim_initial_guess = '';

%% Options structure.
default_options.periodic_solutions = true; % Ensure that velocity and acceleration are continuous between the end and beginning.
default_options.plotting.active = false; % Do we plot intermediate solutions as we go?
default_options.plotting.draw_with_gaps = true; % Do we draw the gaps for low acceleration areas as we go?
default_options.plotting.gap_acceleration_threshold = 1e-2;
default_options.plotting.draw_delay_seconds = 0; % Do we add a delay after drawing so it can be seen better?
default_options.plotting.drawing_points = 2000;
default_options.plotting.draw_initial = true;

% Video writing
default_options.plotting.vid_write_on = false;
default_options.plotting.vid_directory = '../data/videos/raw/';
default_options.plotting.vid_filename = 'spline_optimization.avi';
default_options.plotting.vid_fps = 15;
default_options.plotting.vid_quality = 100;

default_options.verbose = true;

% fmincon options
default_options.fminopts = optimset('fmincon');
default_options.fminopts.Algorithm = 'sqp';
default_options.fminopts.Display = 'iter';
default_options.fminopts.MaxFunEvals = 200000;

%% Return template problem / options.
if nargin == 0
    spline_optim = default_problem;
    return;
elseif nargin == 1 % If single argument given, return either options or probelm structure.
    switch user_problem
        case 'options'
            spline_optim = default_options;
        case 'problem'
            spline_optim = default_problem;
        otherwise
            warning('Unrecognized single variable argument given: %s. Returning problem structure instead.', usr_problem);
            spline_optim = default_problem;
    end
    return;
end

%% Merge user and default options.
problem = mergeOptions(default_problem, user_problem, 'Problem struct');
options = mergeOptions(default_options, user_options, 'Options struct');

validateattributes(problem.pinned_breaks, {'single', 'double'}, {'real', 'vector', 'increasing'});
validateattributes(problem.pinned_knots, {'single', 'double'}, {'real', 'nonempty'});
validateattributes(problem.adjacent_segment_product_scaling, {'numeric'}, {'real', 'scalar', 'positive', '<=', 1});
validateattributes(problem.polys_per_pinned_knot, {'numeric'}, {'scalar', 'integer', 'positive'});

% Fix messed up transposes regarding knots.
if size(problem.pinned_knots, 1) ~= length(problem.pinned_breaks)
    if size(problem.pinned_knots, 2) ~= length(problem.pinned_breaks)
        error('Input knots and breaks do not have a matching number of elements.');
    else
        problem.pinned_knots = problem.pinned_knots';
    end
end

if size(problem.pinned_knots, 2) ~= 2 && size(problem.pinned_knots, 3) ~= 3
   error('Input knots must be  2d or 3d points.');
end

if options.verbose
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
    disp('NLP optimization of path polynomials.');
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
end

%% Formulate the problem symbolically TODO: well... don't do that.
num_total_knots = problem.polys_per_pinned_knot*size(problem.pinned_knots, 1) - problem.polys_per_pinned_knot + 1;
num_total_segments = num_total_knots - 1;
t_p = sym('t_p','real');
a = sym('a', [num_total_segments, 4]); % Trajectory polynomial coefficients. Cubic. For X coordinate
b = sym('b', [num_total_segments, 4]); % Trajectory polynomial coefficients. Cubic. For Y coordinate

% We add extra "subknots" in between the supplied ones. This requires
% additional "subbreaks".
extended_breaks = interp1(1:length(problem.pinned_breaks), problem.pinned_breaks, linspace(1,length(problem.pinned_breaks), num_total_knots))';
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
    cost_integral = cost_integral + problem.adjacent_segment_product_scaling * squareness_term;
end

% Enforce that some points MUST pass through pinned positions.
pinned_pts_x = [xst(1:problem.polys_per_pinned_knot:end); xe(end)] == problem.pinned_knots(:,1);
pinned_pts_y = [yst(1:problem.polys_per_pinned_knot:end); ye(end)] == problem.pinned_knots(:,2);

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

if options.periodic_solutions
    x_constraints = [x_constraints; vxe(end) == vxst(1)]; % Initial velocity matches final velocity (periodic condition).
    y_constraints = [y_constraints; vye(end) == vyst(1)];
    x_constraints = [x_constraints; axe(end) == axst(1)]; % Initial velocity matches final velocity (periodic condition).
    y_constraints = [y_constraints; aye(end) == ayst(1)];
    
    if options.verbose
        disp('Enforcing periodic constraints.');
    end
else % Otherwise 0 velocity at beginning/end.
    x_constraints = [x_constraints; vxe(end) == 0; vxst(1) == 0];
    y_constraints = [y_constraints; vye(end) == 0; vyst(1) == 0];
    
    if options.verbose
        disp('Enforcing zero velocity end constraints.');
    end
end

x_coeffs = reshape(a.',numel(a),1);
y_coeffs = reshape(b.',numel(b),1);

all_constraints = [x_constraints; y_constraints];
all_coeffs = [x_coeffs; y_coeffs];

effort_cost = 10*sum(all_coeffs(1:4:end).^2 + all_coeffs(2:4:end).^2);
cost_fun = matlabFunction(cost_integral + (1 - problem.adjacent_segment_product_scaling)*effort_cost, 'Vars', {all_coeffs});

[A,B] = equationsToMatrix(all_constraints, all_coeffs);
A_num = eval(A);
B_num = eval(B);


%% Pick what initial guess to use.
if ~ischar(problem.guess_pp) % User provided a guess which should override the optim_initial_guess
    guess_xcoefs = problem.guess_pp.coefs(1:3:end,:)';
    guess_ycoefs = problem.guess_pp.coefs(2:3:end,:)';
    % z-coefs are ignored.
    x0 = [guess_xcoefs(:); guess_ycoefs(:)];
    if options.verbose
        disp('Using provided piecewise polynomial as an initial guess.');
    end
elseif ~ischar(problem.optim_initial_guess) % Use a provided guess.
    x0 = problem.optim_initial_guess;
    if options.verbose
        disp('Using provided vector as an initial guess.');
    end
else % Use a random guess if non was provided in any form.
    x0 = rand(size(A_num,2),1) - 0.5;
    if options.verbose
        disp('No guess provided. Using a random vector with [-0.5, 0.5] range.');
    end
end


%% Set up progress plot if desired.
if options.plotting.active
    options.fminopts.OutputFcn = @update_poly_plot;
    nlp_fig = figure;
    nlp_fig.Color = [1,1,1];
    nlp_fig.Position = [100,0,1000,1000];
    coeffs_init = reshape(x0,[4, length(x0)/4]).';
    x_init_pp = mkpp([tst;te(end)], coeffs_init(1:length(coeffs_init)/2,:));
    y_init_pp = mkpp([tst;te(end)], coeffs_init(length(coeffs_init)/2 + 1:end,:));
    x_init_eval = ppval(x_init_pp, linspace(tst(1), te(end), options.plotting.drawing_points)')';
    y_init_eval = ppval(y_init_pp, linspace(tst(1), te(end), options.plotting.drawing_points)')';
    
    spline_plot = plot(x_init_eval, y_init_eval, 'b.');
    spline_plot.Parent.Visible = 'off';
    
    hold on;
    if options.plotting.draw_initial
        spline_plot_init = plot(x_init_eval, y_init_eval, 'Color', [0.1, 0.1, 0.1, 0.4], 'LineWidth', 0.8, 'LineStyle', '--');
    end
    seg_start_plot = plot(x_init_eval(1), y_init_eval(1), '.g', 'MarkerSize', 20);
    seg_end_plot = plot(x_init_eval(end), y_init_eval(end), '.r', 'MarkerSize', 20); % Plot just the contact areas beginning/ends.
    
    % Set axes so they don't change spontaneously during the optimization.
    daspect([1,1,1]);
    axis([min(problem.pinned_knots(:,1)), max(problem.pinned_knots(:,1)), min(problem.pinned_knots(:,2)), max(problem.pinned_knots(:,2))] + [-1, 1, -1, 1]*max(range(x_init_eval), range(y_init_eval))*0.1 + 0.1*[-1, 1, -1, 1]);
    
    hold off;
    
    % Video recording if desired
    if options.plotting.vid_write_on
        vid_writer = VideoWriter([options.plotting.vid_directory, options.plotting.vid_filename]); % Convert for Slack with 'ffmpeg -i infile.avi youroutput.mp4'
        vid_writer.FrameRate = options.plotting.vid_fps;
        vid_writer.Quality = options.plotting.vid_quality;
        open(vid_writer);
    end
    
end

if ~options.verbose
    options.fminopts.Display = 'off';
end

[sol, ~] = fmincon(cost_fun, x0, [],[], A_num, B_num,[],[],[], options.fminopts);

if options.plotting.active && options.plotting.vid_write_on
    close(vid_writer);
end

coeff_sol = reshape(sol,[4, length(sol)/4]).'; % Change from vector to matrix.
ppx = mkpp([tst; te(end)], coeff_sol(1:length(coeff_sol)/2,:));
ppy = mkpp([tst; te(end)], coeff_sol(length(coeff_sol)/2 + 1:end,:));

spline_optim = spline_concat_in_dimension(ppx ,ppy); % Combine the separate ppx and ppy piecewise polys into one.

%% If enabled, this function gets called every iteration to do plotting.
    function stop = update_poly_plot(x, optimValues, state, varargin)
        %% Evaluate current solution.
        coeffs_progress = reshape(x,[4, length(x)/4]).';
        x_progress = mkpp([tst;te(end)], coeffs_progress(1:length(coeffs_progress)/2,:));
        y_progress = mkpp([tst;te(end)], coeffs_progress(length(coeffs_progress)/2 + 1:end,:));
        progress_spline = spline_concat_in_dimension(x_progress, y_progress);
        
        %% Do we break the line (add gaps) when a section has low acceleration?
        if options.plotting.draw_with_gaps
            % Split the fully piecewise polynomial into sections. Makes a separate
            % piecewise polynomial, starting at time 0, for each contact region.
            [~, ~, contact_polys, ~] = find_zero_accel_breaks_from_pos_pp(progress_spline, 3, options.plotting.gap_acceleration_threshold);
            
            pts_per_seg = floor(options.plotting.drawing_points/length(contact_polys));
            num_pts = pts_per_seg * length(contact_polys); % Takes care of pts per not dividing well.
            
            contact_eval = zeros(num_pts,3);
            contact_beginnings = zeros(length(contact_polys),3);
            contact_ends = zeros(length(contact_polys),3);
            
            % Check each individual section.
            for k = 1:length(contact_polys)
                contact_segment_eval = ppval(contact_polys(k), linspace(contact_polys(k).breaks(1), ...
                    contact_polys(k).breaks(end), pts_per_seg))';
                contact_eval(pts_per_seg * (k - 1) + 1:pts_per_seg * k,:) = contact_segment_eval;
                contact_beginnings(k, :) = contact_segment_eval(1,:);
                contact_ends(k, :) = contact_segment_eval(end,:);
            end
            
            spline_plot.XData = contact_eval(:,1);
            spline_plot.YData = contact_eval(:,2);
            
            % Plot the boundaries as dots.
            seg_start_plot.XData = contact_beginnings(:,1);
            seg_start_plot.YData = contact_beginnings(:,2);
            seg_end_plot.XData = contact_ends(:,1);
            seg_end_plot.YData = contact_ends(:,2);
            hold off;
            
        else % Otherwise just plot as a continuous line.
            progress_spline_eval = ppval(progress_spline, linspace(tst(1), te(end), 100))';
            spline_plot.XData = progress_spline_eval(:,1);
            spline_plot.YData = progress_spline_eval(:,2);
        end
        
        if options.plotting.draw_delay_seconds ~= 0
            pause(options.plotting.draw_delay_seconds);
        end
        drawnow;
        
        if options.plotting.vid_write_on
            writeVideo(vid_writer, getframe(nlp_fig));
        end
        
        stop = false; % This function can technically end the optimization although we never want to.
    end
end

