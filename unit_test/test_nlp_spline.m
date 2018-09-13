function exceptions = test_nlp_spline()
% TEST_NLP_SPLINE Unit tests for nlp_spline
% These tests should be expanded to all features of nlp_spline. I have
% relatively high confidence in that code, so I'm prioritizing others right
% now.
%
%   exceptions = TEST_NLP_SPLINE()
%
%   Inputs: <none>
%   Outputs:
%       `exceptions` -- Cell array of exceptions generated while running
%       sub-tests.
%
%   See also NLP_SPLINE, TEST_ALL.
%

tolerance = 1e-10;

theader('Testing nlp_spline.');

knots = [0,0;
    1, 1;
    0, 2;
    -1, 1;
    1, -1;
    0, -2;
    -1, -1;
    0, 0];

breaks = linspace(0, 10, size(knots,1))';

prob = nlp_spline('problem');
prob.pinned_breaks = breaks;
prob.pinned_knots = knots;

opt = nlp_spline('options');
opt.fminopts.Display = 'none';
opt.verbose = false;

nlp_per = nlp_spline(prob, opt);
opt.periodic_solutions = false;

nlp_zero = nlp_spline(prob, opt);
all_pps = [nlp_per, nlp_zero];

exceptions = {};
exceptions{end+1} = do_test(@knot_points_preserved);
exceptions{end+1} = do_test(@end_conditions_met);

    function knot_points_preserved()
        tname('Knots preserved after optimization');
        for i = 1:length(all_pps)
            evals = ppval(all_pps(i), breaks)';
            assert_near(evals(:,1:2), knots(:,1:2), tolerance, 'Knots of optimized spline do not match originals.');
            assert(length(all_pps(i).breaks) ==  length(breaks) + 2*(length(breaks) - 1));
            assert(all_pps(i).order == 4);
            assert(all_pps(i).dim == 3);
        end
    end

    function end_conditions_met()
        tname('End point conditions met');
        
        pos = nlp_per;
        vel = fnder(pos, 1);
        accel = fnder(vel, 1);
        assert_near(ppval(vel, vel.breaks(1)), ppval(vel, vel.breaks(end)), tolerance, 'End velocities do not match.');
        assert_near(ppval(accel, accel.breaks(1)), ppval(accel, accel.breaks(end)), tolerance, 'End accelerations do not match.');

        pos = nlp_zero;
        vel = fnder(pos, 1);
        accel = fnder(vel, 1);
        assert_near(ppval(vel, vel.breaks(1)), [0; 0; 0], tolerance, 'Start vel does not match zero.');
        %assert_near(ppval(accel, accel.breaks(1)), 0, tolerance, 'Start accel does not match zero.');
        assert_near(ppval(vel, vel.breaks(end)), [0; 0; 0], tolerance, 'End vel does not match zero.');
        %assert_near(ppval(accel, accel.breaks(end)), 0, tolerance, 'End accel does not match zero.');
    end
end

