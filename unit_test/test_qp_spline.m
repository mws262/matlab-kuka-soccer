function exceptions = test_qp_spline()
% TEST_QP_SPLINE Unit tests for qp_spline.
%
%   exceptions = TEST_QP_SPLINE()
%
%   Inputs: <none>
%   Outputs:
%       `exceptions` -- Cell array of exceptions generated while running
%       sub-tests.
%
%   See also QP_SPLINE, TEST_ALL.
%

tolerance = 1e-10;

theader('Testing qp_spline.');

knots = [0,0;
    1, 1;
    0, 2;
    -1, 1;
    1, -1;
    0, -2;
    -1, -1;
    0, 0];

breaks = linspace(0, 10, size(knots,1))';

% Make a bunch with different end conditions and number of added segments.
no_added_periodic = qp_spline(breaks, knots, 1, 'periodic');
two_added_periodic = qp_spline(breaks, knots, 3, 'periodic');
two_added_vel = qp_spline(breaks, knots, 3, 'velocities');
two_added_zero = qp_spline(breaks, knots, 3, 'zero');
all_pps = [no_added_periodic, two_added_periodic, two_added_vel, two_added_zero];

exceptions = {};
exceptions{end+1} = do_test(@knot_points_preserved);
exceptions{end+1} = do_test(@end_conditions_met);

    function knot_points_preserved
        tname('Knots preserved after optimization');
        for i = 1:length(all_pps)
           evals = ppval(all_pps(i), breaks)';
           assert_near(evals(:,1:2), knots(:,1:2), tolerance, 'Knots of optimized spline do not match originals.');
        end
        
        assert(length(no_added_periodic.breaks) == length(breaks));
        assert(length(two_added_periodic.breaks) == length(breaks) + 2*(length(breaks) - 1));
        assert(length(two_added_vel.breaks) ==  length(breaks) + 2*(length(breaks) - 1));
        assert(length(two_added_zero.breaks) ==  length(breaks) + 2*(length(breaks) - 1));
    end

    function end_conditions_met()
        tname('End point conditions met');
        
        pos = no_added_periodic;
        vel = fnder(pos, 1);
        accel = fnder(vel, 1);
        assert_near(ppval(vel, vel.breaks(1)), ppval(vel, vel.breaks(end)), tolerance, 'End velocities do not match.');
        assert_near(ppval(accel, accel.breaks(1)), ppval(accel, accel.breaks(end)), tolerance, 'End accelerations do not match.');

        pos = two_added_periodic;
        vel = fnder(pos, 1);
        accel = fnder(vel, 1);
        assert_near(ppval(vel, vel.breaks(1)), ppval(vel, vel.breaks(end)), tolerance, 'End velocities do not match.');
        assert_near(ppval(accel, accel.breaks(1)), ppval(accel, accel.breaks(end)), tolerance, 'End accelerations do not match.');

        pos = two_added_vel;
        vel = fnder(pos, 1);
        assert_near(ppval(vel, vel.breaks(1)), ppval(vel, vel.breaks(end)), tolerance, 'End velocities do not match.');

        pos = two_added_zero;
        vel = fnder(pos, 1);
        accel = fnder(vel, 1);
        assert_near(ppval(vel, vel.breaks(1)), 0, tolerance, 'Start vel does not match zero.');
        assert_near(ppval(accel, accel.breaks(1)), 0, tolerance, 'Start accel does not match zero.');
        assert_near(ppval(vel, vel.breaks(end)), 0, tolerance, 'End vel does not match zero.');
        assert_near(ppval(accel, accel.breaks(end)), 0, tolerance, 'End accel does not match zero.');
    end
end

