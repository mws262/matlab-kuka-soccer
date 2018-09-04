function run_from_knots(knots, breaks, segs_between, vid_filename)
%% QP version
qp_sol = qp_spline(breaks, knots', segs_between);

%% NLP version
nlp_problem = nlp_spline('problem');
nlp_problem.pinned_breaks = breaks;
nlp_problem.pinned_knots = knots';
nlp_problem.polys_per_pinned_knot = segs_between;
% nlp_problem.guess_pp = qp_sol;

nlp_opts = nlp_spline('options');
nlp_opts.plotting.active = true;
nlp_opts.plotting.vid_write_on = true;
nlp_opts.plotting.vid_filename = vid_filename;
nlp_opts.plotting.vid_directory = ['../', nlp_opts.plotting.vid_directory];
nlp_opts.plotting.vid_fps = 8;

nlp_sol = nlp_spline(nlp_problem, nlp_opts);

visualize_spline_with_gaps(qp_sol, nlp_sol);

end

