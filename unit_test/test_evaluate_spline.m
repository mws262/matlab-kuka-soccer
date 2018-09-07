function exceptions = test_evaluate_spline()
tolerance = 1e-15;

theader('Testing evaluate_spline.');

tend = 1;
ball_radius = 1;
coefs = [4,3,2,1; 6,3,1,-1; 0 0 0 ball_radius];
pos_pp = mkpp([0, tend], coefs, 3);

num_pts = 10;

[tspan, posspan, velspan, accelspan, omegaspan, quatspan, ...
    world_contact_loc_desired_fcn, contact_loc_desired_rel_com_fcn] =...
    evaluate_spline(pos_pp, ball_radius, num_pts);

exceptions = {};
exceptions{end+1} = do_test(@linear_states_eval);
exceptions{end+1} = do_test(@rotational_states_eval);
exceptions{end+1} = do_test(@contact_location_pos);

    function linear_states_eval()
        tname('Linear states eval.');
        % Make sure time spans match
        assert_near(linspace(0, tend, num_pts)', tspan, tolerance, 'Tspan does not match.');
        
        % Check some evaluated positions.
        assert_near(posspan(1,:), coefs(:,end)', tolerance, 'posspan does not match.');
        assert_near(posspan(end,:), sum(coefs,2)', tolerance, 'posspan does not match.');

        % Check some evaluated velocities.
        assert_near(velspan(1,:), coefs(:,end - 1)', tolerance, 'velspan does not match.');
        assert_near(velspan(end,:), sum(coefs.*[3,2,1,0],2)', tolerance, 'velspan does not match.');

        % Check some evaluated accelerations.
        assert_near(accelspan(1,:), 2 * coefs(:,end - 2)', tolerance, 'accelspan does not match.');
        assert_near(accelspan(end,:), sum(coefs.*[6,2,0,0],2)', tolerance, 'accelspan does not match.');
    end

    function rotational_states_eval()
        tname('Rotational states eval.');
        assert_near(omegaspan, [-velspan(:,2)/ball_radius, velspan(:,1)/ball_radius, zeros(length(tspan),1)], tolerance, 'omegaspan does not match.');
        assert_near(abs(quatspan(1,:)), [1 0 0 0], tolerance, 'initial quaternion does not match.');

        % Not going to check all of quatspan due to already having a unit
        % test for quatintegrate.
    end

    function contact_location_pos()
        tname('Contact locations eval.');
        %% For zero angle both world and local
        zero_angle = contact_loc_desired_rel_com_fcn(zeros(size(tspan)));
        norm_accel = accelspan./sqrt(sum(accelspan.*accelspan,2));
        assert_near(zero_angle, -ball_radius*norm_accel, tolerance, 'Ball contact location does not match.');
        zero_angle = world_contact_loc_desired_fcn(zeros(size(tspan)));
        assert_near(zero_angle, -ball_radius*norm_accel + posspan, tolerance, 'Ball contact location does not match.');

        %% For pi/2 both world and local.
        right_angle = contact_loc_desired_rel_com_fcn(pi/2*ones(size(tspan)));
        assert_near(right_angle, zeros(size(accelspan)) + [0, 0, ball_radius], tolerance, 'Ball contact location does not match.');
        right_angle = world_contact_loc_desired_fcn(pi/2*ones(size(tspan)));
        assert_near(right_angle, zeros(size(accelspan)) + [0, 0, ball_radius] + posspan, tolerance, 'Ball contact location does not match.');
    end
end

