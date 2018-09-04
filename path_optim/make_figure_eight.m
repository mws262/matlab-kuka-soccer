close all;clear all;
%% Shape parameters
file_name = 'figure_eight_centered.mat';
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
    0, 0, height];

num_knots = size(knots,2);
breaks = linspace(0, time_scaling, num_knots);

%% QP version
qp_fig8_spline = qp_spline(breaks, knots, segs_between);

%% NLP version
nlp_problem = nlp_spline('problem');
nlp_problem.pinned_breaks = breaks;
nlp_problem.pinned_knots = knots;
nlp_problem.polys_per_pinned_knot = segs_between;
nlp_problem.guess_pp = qp_fig8_spline;

nlp_opts = nlp_spline('options');

nlp_fig8_spline = nlp_spline(nlp_problem, nlp_opts); % 3 segments in between knots. Highest weight on "forcing" linear sections.

nlp_fig8_spline.coefs = nlp_fig8_spline.coefs * scale; % For some reason, adjusting scale and offset before the optimization does weird things. Maybe because of convergence tolerances?
nlp_fig8_spline.coefs(1:3:end,end) = nlp_fig8_spline.coefs(1:3:end,end) + offset(1);
nlp_fig8_spline.coefs(2:3:end,end) = nlp_fig8_spline.coefs(2:3:end,end) + offset(2);
visualize_spline_with_gaps(qp_fig8_spline, nlp_fig8_spline);

save(['../data/', file_name], 'nlp_fig8_spline');


