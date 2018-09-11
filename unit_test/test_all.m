function test_all()
% TEST_ALL Runs the full set of unit tests.
%
%   TEST_ALL()
%
%   Inputs: <none>
%   Outputs: <none>
%
%   See also SETUP, DO_TEST, REPORT_EXCEPTIONS, ASSERT, VALIDATEATTRIBUTES.
%

exceptions = {};

ex = do_test(@test_general_interpolator);
exceptions = [exceptions, ex];

ex = do_test(@test_check_pt_inside_mesh);
exceptions = [exceptions, ex];

ex = do_test(@test_skew);
exceptions = [exceptions, ex];

ex = do_test(@test_quatintegrate);
exceptions = [exceptions, ex];

ex = do_test(@test_modified_gram_schmidt);
exceptions = [exceptions, ex];

ex = do_test(@test_mergeOptions);
exceptions = [exceptions, ex];

ex = do_test(@test_make_ball);
exceptions = [exceptions, ex];

ex = do_test(@test_dynamics_autogen);
exceptions = [exceptions, ex];

ex = do_test(@test_evaluate_spline);
exceptions = [exceptions, ex];

ex = do_test(@test_spline_concat_in_dimension);
exceptions = [exceptions, ex];

ex = do_test(@test_cmaes);
exceptions = [exceptions, ex];

ex = do_test(@test_point2trimesh_with_normals);
exceptions = [exceptions, ex];

ex = do_test(@test_get_rotation_from_vecs);
exceptions = [exceptions, ex];

ex = do_test(@test_qp_spline);
exceptions = [exceptions, ex];

ex = do_test(@test_nlp_spline);
exceptions = [exceptions, ex];

ex = do_test(@test_find_mesh_contact_tform);
exceptions = [exceptions, ex];

ex = do_test(@test_get_mesh_data);
exceptions = [exceptions, ex];

report_exceptions(exceptions);
end