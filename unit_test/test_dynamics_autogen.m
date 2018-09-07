function exceptions = test_dynamics_autogen()
%TEST_DYNAMICS_AUTOGEN Mostly uses easy-to-reason-about special cases for 
%testing.

tolerance = 1e-15;

theader('Testing derived dynamics expressions.');
exceptions = {};
exceptions{end+1} = do_test(@contact_arc_body_center);
exceptions{end+1} = do_test(@contact_arc_world_center);
exceptions{end+1} = do_test(@angular_rate_from_vel);
exceptions{end+1} = do_test(@equator_contact_vel);
% exceptions{end+1} = do_test(@contact_unit_vecs);
exceptions{end+1} = do_test(@world_contact_velocity);

    function contact_arc_body_center()
        tname('Position on arc relative to ball center.');
        % Should be opposite the acceleration.
        % Y-direction
        R = 1;
        ax = 0;
        ay = 1;
        theta = 0;
        pos = contact_arc_centered_fcn(R,ax,ay,theta);
        assert_near(pos, [0, -R, 0], tolerance, 'Bad contact_arc_body_center calculation.');
        
        % x-y direction
        R = 1;
        ax = 1;
        ay = 1;
        theta = 0;
        pos = contact_arc_centered_fcn(R,ax,ay,theta);
        assert_near(pos, [-R/sqrt(2), -R/sqrt(2), 0], tolerance, 'Bad contact_arc_body_center calculation.');
        
        % x-y direction w/ some angle.
        R = 1;
        ax = 1;
        ay = 1;
        theta = pi/7;
        pos = contact_arc_centered_fcn(R,ax,ay,theta);
        assert_near(pos, [-R*cos(theta)/sqrt(2), -R*cos(theta)/sqrt(2), R*sin(theta)], tolerance, 'Bad contact_arc_body_center calculation.');
    end

    function contact_arc_world_center()
        tname('Position on arc rel world.');
        % Should be opposite the acceleration.
        % Y-direction
        R = 1;
        ax = 0;
        ay = 1;
        rx = -1;
        ry = 2;
        theta = 0;
        pos = contact_arc_fcn(R,ax,ay,rx,ry,theta);
        assert_near(pos, [0, -R, 0] + [rx, ry, R], tolerance, 'Bad contact_arc_world_center calculation.');
        
        % x-y direction
        R = 1;
        ax = 1;
        ay = 1;
        rx = 5;
        ry = 3;
        theta = 0;
        pos = contact_arc_fcn(R,ax,ay,rx,ry,theta);
        assert_near(pos, [-R/sqrt(2), -R/sqrt(2), 0] + [rx, ry, R], tolerance, 'Bad contact_arc_world_center calculation.');
        
        % x-y direction w/ some angle.
        R = 1;
        ax = 1;
        ay = 1;
        rx = -5;
        ry = 8;
        theta = pi/7;
        pos = contact_arc_fcn(R,ax,ay,rx,ry,theta);
        assert_near(pos, [-R*cos(theta)/sqrt(2), -R*cos(theta)/sqrt(2), R*sin(theta)] + [rx, ry, R], tolerance, 'Bad contact_arc_world_center calculation.');
    end

    function angular_rate_from_vel()
        tname('Angular rate from linear vel.');
        rx1 = angular_rate_wx_fcn(1,-1);
        assert_near(rx1, 1, tolerance, 'angular_rate_from_vel bad in x direction');
        ry1 = angular_rate_wy_fcn(1,1);
        assert_near(ry1, 1, tolerance, 'angular_rate_from_vel bad in y direction');
    end

    function equator_contact_vel()
        tname('Equator contact velocity.');
        %% On x axis.
        vx = 1;
        vy = 0;
        ax = 0;
        ay = -1;
        contact_vel = equator_contact_velocity_fcn(ax,ay,vx,vy);
        assert_near(contact_vel, [vx,0,0], tolerance, 'bad equator contact_vel');
        
        vx = 1;
        vy = 0;
        ax = 0;
        ay = 1;
        contact_vel = equator_contact_velocity_fcn(ax,ay,vx,vy);
        assert_near(contact_vel, [vx,0,0], tolerance, 'bad equator contact_vel');
        
        vx = 1;
        vy = 0;
        ax = 1;
        ay = 0;
        contact_vel = equator_contact_velocity_fcn(ax,ay,vx,vy);
        assert_near(contact_vel, [vx, 0, vx], tolerance, 'bad equator contact_vel');
        
        vx = 1;
        vy = 0;
        ax = -1;
        ay = 0;
        contact_vel = equator_contact_velocity_fcn(ax,ay,vx,vy);
        assert_near(contact_vel, [vx, 0, -vx], tolerance, 'bad equator contact_vel');
        
        %% On y axis
        vx = 0;
        vy = 1;
        ax = 1;
        ay = 0;
        contact_vel = equator_contact_velocity_fcn(ax,ay,vx,vy);
        assert_near(contact_vel, [0,vy,0], tolerance, 'bad equator contact_vel');
        
        vx = 0;
        vy = 1;
        ax = -1;
        ay = 0;
        contact_vel = equator_contact_velocity_fcn(ax,ay,vx,vy);
        assert_near(contact_vel, [0,vy,0], tolerance, 'bad equator contact_vel');
        
        vx = 0;
        vy = 1;
        ax = 0;
        ay = 1;
        contact_vel = equator_contact_velocity_fcn(ax,ay,vx,vy);
        assert_near(contact_vel, [0, vy, vy], tolerance, 'bad equator contact_vel');
        
        vx = 0;
        vy = 1;
        ax = 0;
        ay = -1;
        contact_vel = equator_contact_velocity_fcn(ax,ay,vx,vy);
        assert_near(contact_vel, [0, vy, -vy], tolerance, 'bad equator contact_vel');
    end

%     function contact_unit_vecs()
%         tname('Contact plane unit vecs.');
%         
%         % Unit vectors of contact plane touching somewhere on the sphere.
%         % NOTE: these go singular.
%         ax = -1;
%         ay = 0;
%         theta = 0;
%         i_unit = isurf_fcn(ax,ay,theta);
%         j_unit = jsurf_fcn(ax,ay,theta);
%         assert_near(i_unit, [0, -1, 0], tolerance, 'bad contact_unit_vecs');
%         assert_near(j_unit, [0, 0, 1], tolerance, 'bad contact_unit_vecs');
%         
%         ax = 1;
%         ay = 0;
%         theta = 0;
%         i_unit = isurf_fcn(ax,ay,theta);
%         j_unit = jsurf_fcn(ax,ay,theta);
%         assert_near(i_unit, [0, 1, 0], tolerance, 'bad contact_unit_vecs');
%         assert_near(j_unit, [0, 0, 1], tolerance, 'bad contact_unit_vecs');
%         
%         ax = 0;
%         ay = 1;
%         theta = 0;
%         i_unit = isurf_fcn(ax,ay,theta);
%         j_unit = jsurf_fcn(ax,ay,theta);
%         assert_near(i_unit, [-1, 0, 0], tolerance, 'bad contact_unit_vecs');
%         assert_near(j_unit, [0, 0, 1], tolerance, 'bad contact_unit_vecs');
%         
%         ax = 0;
%         ay = -1;
%         theta = 0;
%         i_unit = isurf_fcn(ax,ay,theta);
%         j_unit = jsurf_fcn(ax,ay,theta);
%         assert_near(i_unit, [1, 0, 0], tolerance, 'bad contact_unit_vecs');
%         assert_near(j_unit, [0, 0, 1], tolerance, 'bad contact_unit_vecs');
%         
%         ax = -1;
%         ay = 0;
%         theta = pi/2;
%         i_unit = isurf_fcn(ax,ay,theta);
%         j_unit = jsurf_fcn(ax,ay,theta);
%         assert_near(i_unit, [0, -1, 0], tolerance, 'bad contact_unit_vecs');
%         assert_near(j_unit, [-1, 0, 0], tolerance, 'bad contact_unit_vecs');
%         
%         ax = 1;
%         ay = 0;
%         theta = pi/2;
%         i_unit = isurf_fcn(ax,ay,theta);
%         j_unit = jsurf_fcn(ax,ay,theta);
%         assert_near(i_unit, [0, 1, 0], tolerance, 'bad contact_unit_vecs');
%         assert_near(j_unit, [1, 0, 0], tolerance, 'bad contact_unit_vecs');
%         
%     end

    function world_contact_velocity()
        tname('Contact point velocity.');
        %% FOR 0 THETA
        %% On x axis.
        vx = 1;
        vy = 0;
        ax = 0;
        ay = -1;
        theta = 0;
        contact_vel = world_contact_velocity_fcn(ax,ay,theta,vx,vy);
        assert_near(contact_vel, [vx,0,0]', tolerance, 'bad contact_vel');
        
        vx = 1;
        vy = 0;
        ax = 0;
        ay = 1;
        theta = 0;
        contact_vel = world_contact_velocity_fcn(ax,ay,theta,vx,vy);
        assert_near(contact_vel, [vx,0,0]', tolerance, 'bad contact_vel');
        
        vx = 1;
        vy = 0;
        ax = 1;
        ay = 0;
        theta = 0;
        contact_vel = world_contact_velocity_fcn(ax,ay,theta,vx,vy);
        assert_near(contact_vel, [vx, 0, vx]', tolerance, 'bad contact_vel');
        
        vx = 1;
        vy = 0;
        ax = -1;
        ay = 0;
        theta = 0;
        contact_vel = world_contact_velocity_fcn(ax,ay,theta,vx,vy);
        assert_near(contact_vel, [vx, 0, -vx]', tolerance, 'bad contact_vel');
        
        %% On y axis
        vx = 0;
        vy = 1;
        ax = 1;
        ay = 0;
        theta = 0;
        contact_vel = world_contact_velocity_fcn(ax,ay,theta,vx,vy);
        assert_near(contact_vel, [0,vy,0]', tolerance, 'bad contact_vel');
        
        vx = 0;
        vy = 1;
        ax = -1;
        ay = 0;
        theta = 0;
        contact_vel = world_contact_velocity_fcn(ax,ay,theta,vx,vy);
        assert_near(contact_vel, [0,vy,0]', tolerance, 'bad contact_vel');
        
        vx = 0;
        vy = 1;
        ax = 0;
        ay = 1;
        theta = 0;
        contact_vel = world_contact_velocity_fcn(ax,ay,theta,vx,vy);
        assert_near(contact_vel, [0, vy, vy]', tolerance, 'bad contact_vel');
        
        vx = 0;
        vy = 1;
        ax = 0;
        ay = -1;
        theta = 0;
        contact_vel = world_contact_velocity_fcn(ax,ay,theta,vx,vy);
        assert_near(contact_vel, [0, vy, -vy]', tolerance, 'bad contact_vel');
        
        %% FOR pi/2 angle
        %% On x axis.
        vx = 1;
        vy = 0;
        ax = 0;
        ay = -1;
        theta = pi/2;
        contact_vel = world_contact_velocity_fcn(ax,ay,theta,vx,vy);
        assert_near(contact_vel, [2*vx,0,0]', tolerance, 'bad contact_vel');
        
        vx = 1;
        vy = 0;
        ax = 0;
        ay = 1;
        theta = pi/2;
        contact_vel = world_contact_velocity_fcn(ax,ay,theta,vx,vy);
        assert_near(contact_vel, [2*vx,0,0]', tolerance, 'bad contact_vel');
        
        vx = 1;
        vy = 0;
        ax = 1;
        ay = 0;
        theta = pi/2;
        contact_vel = world_contact_velocity_fcn(ax,ay,theta,vx,vy);
        assert_near(contact_vel, [2*vx, 0, 0]', tolerance, 'bad contact_vel');
        
        vx = 1;
        vy = 0;
        ax = -1;
        ay = 0;
        theta = pi/2;
        contact_vel = world_contact_velocity_fcn(ax,ay,theta,vx,vy);
        assert_near(contact_vel, [2*vx, 0, 0]', tolerance, 'bad contact_vel');
        
        %% On y axis
        vx = 0;
        vy = 1;
        ax = 1;
        ay = 0;
        theta = pi/2;
        contact_vel = world_contact_velocity_fcn(ax,ay,theta,vx,vy);
        assert_near(contact_vel, [0,2*vy,0]', tolerance, 'bad contact_vel');
        
        vx = 0;
        vy = 1;
        ax = -1;
        ay = 0;
        theta = pi/2;
        contact_vel = world_contact_velocity_fcn(ax,ay,theta,vx,vy);
        assert_near(contact_vel, [0,2*vy,0]', tolerance, 'bad contact_vel');
        
        vx = 0;
        vy = 1;
        ax = 0;
        ay = 1;
        theta = pi/2;
        contact_vel = world_contact_velocity_fcn(ax,ay,theta,vx,vy);
        assert_near(contact_vel, [0, 2*vy, 0]', tolerance, 'bad contact_vel');
        
        vx = 0;
        vy = 1;
        ax = 0;
        ay = -1;
        theta = pi/2;
        contact_vel = world_contact_velocity_fcn(ax,ay,theta,vx,vy);
        assert_near(contact_vel, [0, 2*vy, 0]', tolerance, 'bad contact_vel');
    end
end
