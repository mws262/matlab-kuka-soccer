close all; clear all;

%% Shape parameters
file_name = 'triangle_position_spline_no_offset.mat';
offset = [- 0.1, 0.0];
scale = 0.15;
knots = [ 0, sqrt(3), 0; 1, 0, 0; -1, 0, 0; 0, sqrt(3), 0];
breaks = linspace(0, 10, size(knots,1))';

%% QP version.
segs_between = 3;
qp_triangle_spline = qp_spline(breaks, knots, segs_between);

%% NLP version.
nlp_problem = nlp_spline('problem');
nlp_problem.pinned_breaks = breaks;
nlp_problem.pinned_knots = knots;
nlp_problem.polys_per_pinned_knot = segs_between;
nlp_problem.guess_pp = qp_triangle_spline;

nlp_opts = nlp_spline('options');

nlp_triangle_spline = nlp_spline(nlp_problem, nlp_opts); % 3 segments in between knots. Highest weight on "forcing" linear sections.

nlp_triangle_spline.coefs = nlp_triangle_spline.coefs * scale; % For some reason, adjusting scale and offset before the optimization does weird things. Maybe because of convergence tolerances?
nlp_triangle_spline.coefs(1:3:end,end) = nlp_triangle_spline.coefs(1:3:end,end) + offset(1);
nlp_triangle_spline.coefs(2:3:end,end) = nlp_triangle_spline.coefs(2:3:end,end) + offset(2);

visualize_spline_with_gaps(qp_triangle_spline, nlp_triangle_spline);

save(['../data/', file_name], 'nlp_triangle_spline');

