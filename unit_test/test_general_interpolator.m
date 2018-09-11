function exceptions = test_general_interpolator()
%TEST_GENERAL_INTERPOLATOR Test the general interpolator function.
tolerance = 1e-15;

theader('Testing general_interpolator.');
exceptions = {};
exceptions{end+1} = do_test(@single_dim);
exceptions{end+1} = do_test(@two_d);
exceptions{end+1} = do_test(@three_d);


    function single_dim()
        %% Single dimension
        tname('1D case');
        single_dim_interp = general_interpolator(1);
        tspan1 = linspace(-1, 5, 12)';
        dspan1 = rand(size(tspan1));
        single_dim_interp.add_to_end_at_time(tspan1, dspan1);
        assert_near(single_dim_interp.get_at_time(3), interp1(tspan1, dspan1, 3), tolerance, 'general_interpolator does not agree with interp1 in 1d case.');
        assert_near(single_dim_interp.get_at_time(-0.99), interp1(tspan1, dspan1, -0.99), tolerance, 'general_interpolator does not agree with interp1 in 1d case.');
        assert_near(single_dim_interp.get_at_time(-1), interp1(tspan1, dspan1, -1), tolerance, 'general_interpolator does not agree with interp1 in 1d case.');
        assert_near(single_dim_interp.get_at_time(5), interp1(tspan1, dspan1, 5), tolerance, 'general_interpolator does not agree with interp1 in 1d case.');
        
        % Add another element.
        added_ele_time = 6;
        added_ele_val = 4;
        single_dim_interp.add_to_end_at_time(added_ele_time, added_ele_val);
        assert_near(single_dim_interp.get_at_time(5.5), (dspan1(end) + added_ele_val)/2, tolerance, 'adding a single element to the end of an existing dataset produced bad interpoation values in between old and new in 1d case.');
        
        % Adding overlapping data should throw.
        assert_error(@()(single_dim_interp.add_to_end_at_time(0, 1)));
        assert_error(@()(single_dim_interp.add_to_end_at_time(-2, 1)));
        assert_error(@()(single_dim_interp.add_to_end_at_time(-1, 1)));
        assert_error(@()(single_dim_interp.add_to_end_at_time(6, 1)));
        assert_error(@()(single_dim_interp.add_to_end_at_time(12, NaN)));
        assert_error(@()(single_dim_interp.add_to_end_at_time(NaN, 12)));
        % Bad query times should throw.
        assert_error(@()(single_dim_interp.get_at_time(7)));
        assert_error(@()(single_dim_interp.get_at_time(-2)));        
        assert_error(@()(single_dim_interp.get_at_time(NaN)));
        assert_error(@()(single_dim_interp.get_at_time([1,2])));

    end

    function two_d()
        %% 2D
        tname('2D case');
        dim = 4;
        two_d_interp = general_interpolator(dim);
        tspan2 = linspace(-1, 5, 12)';
        dspan2 = rand(size(tspan2,1),dim);
        two_d_interp.add_to_end_at_time(tspan2,dspan2);
        
        assert_near(two_d_interp.get_at_time(3), interp1(tspan2, dspan2, 3), tolerance, 'general_interpolator does not agree with interp1 in 2d case.');
        assert_near(two_d_interp.get_at_time(-0.99), interp1(tspan2, dspan2, -0.99), tolerance, 'general_interpolator does not agree with interp1 in 2d case.');
        assert_near(two_d_interp.get_at_time(-1), interp1(tspan2, dspan2, -1), tolerance, 'general_interpolator does not agree with interp1 in 2d case.');
        assert_near(two_d_interp.get_at_time(5), interp1(tspan2, dspan2, 5), tolerance, 'general_interpolator does not agree with interp1 in 2d case.');
        
        % Add another element.
        added_ele_time = 6;
        added_ele_val = rand(1,dim);
        two_d_interp.add_to_end_at_time(added_ele_time, added_ele_val);
        assert_near(two_d_interp.get_at_time(5.5), (dspan2(end,:) + added_ele_val)/2, tolerance, 'adding a single element to the end of an existing dataset produced bad interpoation values in between old and new in 2d case.');
    end

    function three_d()
        %% 3D
        tname('3D case');
        dim = [3,3];
        three_d_interp = general_interpolator(dim);
        tspan3 = linspace(-1,5,12)';
        dspan3 = rand([3, 3, length(tspan3)]);
        
        three_d_interp.add_to_end_at_time(tspan3,dspan3);
        
        % Check boundaries
        assert_near(three_d_interp.get_at_time(-1), dspan3(:,:,1), tolerance, 'general_interpolator does not agree with interp1 in 3d case.');
        assert_near(three_d_interp.get_at_time(5), dspan3(:,:,end), tolerance, 'general_interpolator does not agree with interp1 in 3d case.');

        % Check some mid time.
        mid_t1 = 1.28;
        lower_idx = find(tspan3 < mid_t1, 1, 'last');
        interp_mid1 = (dspan3(:,:,lower_idx + 1) - dspan3(:,:,lower_idx))*(mid_t1 - tspan3(lower_idx))/(tspan3(lower_idx + 1) - tspan3(lower_idx)) + dspan3(:,:,lower_idx);
        assert_near(three_d_interp.get_at_time(mid_t1), interp_mid1, tolerance, 'general_interpolator does not agree with interp1 in 3d case.');
        
        new_ele = rand(3);
        three_d_interp.add_to_end_at_time(6, new_ele);
        mid_t2 = 5.47;
        interp_mid2 = (new_ele - dspan3(:,:,end))*(mid_t2 - tspan3(end))/(6 - tspan3(end)) + dspan3(:,:,end);
        assert_near(three_d_interp.get_at_time(mid_t2), interp_mid2, tolerance, 'general_interpolator does not agree with interp1 in 3d case.');
        
        
    end

end

