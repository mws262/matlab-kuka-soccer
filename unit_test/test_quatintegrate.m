function exceptions = test_quatintegrate()
%TEST_QUATINTEGRATE
high_tolerance = 1e-12; % For stuff which should be THE SAME minus a little floating point error.
low_tolerance = 1e-3; % For integration where errors are expected to be larger.

theader('Testing quatintegrate.');
exceptions = {};

exceptions{end+1} = do_test(@vector_unitness);
exceptions{end+1} = do_test(@method_difference_closeness);
exceptions{end+1} = do_test(@difference_convergence);

% Quaternions should have a magnitude near 1.
    function vector_unitness()
        tname('Unit-length test.');
        [quatspan_rk4, quatspan_euler] = do_comparative_integration(100);
        assert_near(max(sqrt(sum(quatspan_rk4.*quatspan_rk4,2))), 1, high_tolerance, 'Unexpected deviation from unit-size in rk4 integrated quaternions.');
        assert_near(max(sqrt(sum(quatspan_euler.*quatspan_euler,2))), 1, high_tolerance, 'Unexpected deviation from unit-size in euler integrated quaternions.');

    end

% Both integration schemes should roughly agree.
    function method_difference_closeness()
        tname('RK4 vs. Euler test.');
        [quatspan_rk4, quatspan_euler] = do_comparative_integration(10000);
        int_diff = quatspan_rk4 - quatspan_euler;
        max_norm_diff = max(sqrt(sum(int_diff.*int_diff,2)));
        assert_near(max_norm_diff, 0, low_tolerance, ['Euler integrated and RK4 integrated solutions deviated by too much. Max norm diff: ', num2str(max_norm_diff)]);
    end

% Error is dominated by euler and so with number of steps increased we
% should see roughly linear decreases in the difference between the two.
    function difference_convergence()
        tname('Error convergence test.');
        [quatspan_rk4, quatspan_euler] = do_comparative_integration(1000);
        int_diff = quatspan_rk4 - quatspan_euler;
        max_norm_diff1 = max(sqrt(sum(int_diff.*int_diff,2)));
        [quatspan_rk4, quatspan_euler] = do_comparative_integration(1000*exp(1));
        int_diff = quatspan_rk4 - quatspan_euler;
        max_norm_diff2 = max(sqrt(sum(int_diff.*int_diff,2)));
        [quatspan_rk4, quatspan_euler] = do_comparative_integration(1000*exp(2));
        int_diff = quatspan_rk4 - quatspan_euler;
        max_norm_diff3 = max(sqrt(sum(int_diff.*int_diff,2)));
        
        assert_near(log(max_norm_diff1) - 1, log(max_norm_diff2), 1e-2, 'Difference between euler and RK4 should be close to linear in number of timesteps.');
        assert_near(log(max_norm_diff2) - 1, log(max_norm_diff3), 1e-2, 'Difference between euler and RK4 should be close to linear in number of timesteps.');

    end

    function [quatspan_rk4, quatspan_euler] = do_comparative_integration(num_pts)
        %% Make some generic ball path stuff + states
        % Arc
        tend = 5;
        R = 0.1;
        offset = [0.4;0;0];
        knots = [0,-R*1.5,0; R/2,0,0; 0, R*1.5,0]' + offset;
        
        breaks = linspace(0, tend, size(knots,2));
        pos_pp = spline(breaks, [[0;0;0], knots, [0;0;0]]);
        
        ball_radius = 0.1;
        vel_pp = fnder(pos_pp);
        accel_pp = fnder(vel_pp);
        points = num_pts;
        tspan = linspace(0, tend, points)';
        % Replicate everything if we want to do multiple laps.
        pos_eval = ppval(pos_pp, tspan)';
        vel_eval = ppval(vel_pp, tspan)';
        accel_eval = ppval(accel_pp, tspan)';
        
        % States, etc
        zero_vec_pts = zeros(size(tspan));
        omega = [angular_rate_wx_fcn(ball_radius, vel_eval(:,2)), angular_rate_wy_fcn(ball_radius, vel_eval(:,1)), zero_vec_pts];
        
        rotationQ = [1 0 0 0]; % Quaternion representing ball rotation.
        
        % Run RK4 version.
        quatspan_rk4 = quatintegrate(tspan, rotationQ, omega);
        
        % Run Euler version.
        quatspan_euler = zeros(length(tspan),4);
        quatspan_euler(1,:) = rotationQ;
        for i = 1:length(tspan) - 1
            dt = tspan(i+1) - tspan(i);
            quatspan_euler(i + 1,:) = 0.5 * quatmultiply([0, omega(i,:)], quatspan_euler(i,:)) * dt + quatspan_euler(i,:);
            quatspan_euler(i + 1,:) = quatspan_euler(i + 1,:)/norm(quatspan_euler(i + 1,:));
        end
        quatspan_euler(:,1) = -quatspan_euler(:,1);
    end
end

