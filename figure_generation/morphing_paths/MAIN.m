close all; clear all;
addpath ../../util/;
%% Generates videos and images of the optimization process.

%%%%%%%%%%%% FIGURE 8 %%%%%%%%%%%%%%%%%%%%
%% Shape parameters
segs_between = 3; % One of the most important parameters. Will change the number of extra segments between required knot points.
time_scaling = 5; % Time to make one cycle along the 8.
lobe_length = 0.5; % Parameters to stretch the 8.
lobe_width = 0.2;
lobe_center_offset = 0;
offset = [1,1,0];
scale = 1;
height = 0; % Height of path off the ground. I see no reason to change this.

%% Knots and breaks
knots = [0, 0, height;
    lobe_width, lobe_length/2 + lobe_center_offset, height;
    0, lobe_length, height;
    -lobe_width, lobe_length/2 + lobe_center_offset, height
    0, 0, height;
    lobe_width, -lobe_length/2 - lobe_center_offset, height;
    0, -lobe_length, height;
    -lobe_width, -lobe_length/2 - lobe_center_offset, height;
    0, 0, height]';

num_knots = size(knots,2);
breaks = linspace(0, time_scaling, num_knots);

%% QP version
qp_sol = qp_spline(breaks, knots', segs_between);

%% NLP version
nlp_problem = nlp_spline('problem');
nlp_problem.pinned_breaks = breaks;
nlp_problem.pinned_knots = knots';
nlp_problem.polys_per_pinned_knot = segs_between;

nlp_opts = nlp_spline('options');
nlp_opts.plotting.active = true;
nlp_opts.plotting.vid_write_on = true;
nlp_opts.plotting.draw_initial = false;

nlp_opts.plotting.vid_directory = ['../', nlp_opts.plotting.vid_directory];
nlp_opts.plotting.vid_fps = 8;

% With no initial guess.
nlp_opts.plotting.vid_filename = 'spline_optimization_figure_eight_rand_guess.avi';
nlp_sol = nlp_spline(nlp_problem, nlp_opts);
 
% nlp_problem.guess_pp = qp_sol;
% nlp_opts.plotting.vid_filename = 'spline_optimization_figure_eight_qp_guess.avi';
% nlp_sol = nlp_spline(nlp_problem, nlp_opts);

for i = 3:6 % Can do it for multiple segments. Not interesting above 6
    nlp_problem.polys_per_pinned_knot = i;
    nlp_opts.plotting.draw_initial = true;
    qp_sol = qp_spline(breaks, knots', i); % Gotta rerun QP for this many segs.
    nlp_problem.guess_pp = qp_sol;
    nlp_opts.plotting.vid_filename = ['spline_optimization_figure_eight_qp_guess_', num2str(i), '_segs.avi'];
    nlp_sol = nlp_spline(nlp_problem, nlp_opts);
end

% Also save images
spline_fig = visualize_spline_with_gaps(qp_sol, nlp_sol);
save2pdf('../../data/images/spline_optimization_figure_8.pdf', spline_fig, 600)
% run_from_knots(knots, breaks, 3, vid_filename);

%%%%%%%%%%% TRIANGLE %%%%%%%%%%%%%%%%%%

knots = [ 0, sqrt(3), 0; 1, 0, 0; -1, 0, 0; 0, sqrt(3), 0]';
breaks = linspace(0, 5, size(knots,1))';

%% QP version
qp_sol = qp_spline(breaks, knots', segs_between);

%% NLP version
nlp_problem = nlp_spline('problem');
nlp_problem.pinned_breaks = breaks;
nlp_problem.pinned_knots = knots';
nlp_problem.polys_per_pinned_knot = segs_between;

nlp_opts = nlp_spline('options');
nlp_opts.plotting.active = true;
nlp_opts.plotting.vid_write_on = true;

nlp_opts.plotting.vid_directory = ['../', nlp_opts.plotting.vid_directory];
nlp_opts.plotting.vid_fps = 8;

% With no initial guess.
nlp_opts.plotting.vid_filename = 'spline_optimization_triangle_rand_guess.avi';
nlp_opts.plotting.draw_initial = false;
nlp_sol = nlp_spline(nlp_problem, nlp_opts);

nlp_problem.guess_pp = qp_sol;
nlp_opts.plotting.vid_filename = 'spline_optimization_triangle_qp_guess.avi';
nlp_opts.plotting.draw_initial = true;
nlp_sol = nlp_spline(nlp_problem, nlp_opts);

% Also save images
spline_fig = visualize_spline_with_gaps(qp_sol, nlp_sol);
save2pdf('../../data/images/spline_optimization_triangle.pdf', spline_fig, 600)

%%%%%%%%%%%%%%%%%% TRI %%%%%%%%%%%%%%%%%%%%%%%
 segs_between = 3;
 knots = [0,0,0
      0.6, 1.5, 0.0
      0.5, 2.0, 0.0
      0.45, 1.4, 0.0

      1.0, 0.0, 0.0
      1.5, 1.0, 0.0
      1.8, 0.8, 0.0
      1.55, 0.5, 0.0

      2.0, 0.0, 0.0
      2.53, 0.5, 0.0
      2.5, 1.0, 0.0
      2.47, 0.5, 0.0
      3.0, 0.0, 0.0

      3.5, 0.3, 0
      4, 0.8, 0

      2.5, 1.5, 0
      2.45, 1.45, 0

      2.5, 1.5, 0

      1.5, 1.6, 0
      0.5, 1.5, 0
      -1, 1.2, 0]';
  
  breaks = linspace(0, 20, size(knots,1));
  
%% QP version
qp_sol = qp_spline(breaks, knots', segs_between);

%% NLP version
nlp_problem = nlp_spline('problem');
nlp_problem.pinned_breaks = breaks;
nlp_problem.pinned_knots = knots';
nlp_problem.polys_per_pinned_knot = segs_between;

nlp_opts = nlp_spline('options');
nlp_opts.periodic_solutions = false;
nlp_opts.plotting.active = true;
nlp_opts.plotting.vid_write_on = true;
% nlp_opts.plotting.gap_acceleration_threshold = 0.01;

nlp_opts.plotting.vid_directory = ['../', nlp_opts.plotting.vid_directory];
nlp_opts.plotting.vid_fps = 15;

% With no initial guess.
nlp_opts.plotting.vid_filename = 'spline_optimization_TRI_rand_guess.avi';
nlp_opts.plotting.draw_initial = false;
nlp_sol = nlp_spline(nlp_problem, nlp_opts);

nlp_problem.guess_pp = qp_sol;
nlp_opts.plotting.vid_filename = 'spline_optimization_TRI_qp_guess.avi';
nlp_opts.plotting.draw_initial = true;
nlp_sol = nlp_spline(nlp_problem, nlp_opts);

% Also save images
spline_fig = visualize_spline_with_gaps(qp_sol, nlp_sol);
save2pdf('../../data/images/spline_optimization_TRI.pdf', spline_fig, 600)


