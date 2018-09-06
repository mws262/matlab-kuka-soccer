exceptions = {};

ex = do_test(@test_general_interpolator);
exceptions = [exceptions, ex];

ex = do_test(@test_check_pt_inside_mesh);
exceptions = [exceptions, ex];

ex = do_test(@test_skew);
exceptions = [exceptions, ex];

report_exceptions(exceptions);