function test_general_interpolator()
%TEST_GENERAL_INTERPOLATOR Test the general interpolator function.
tolerance = 1e-15;

theader('Testing general_interpolator');

%% Single dimension
tname('Single dimensional test');
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

%% 2D
tname('2D test');
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


%% 3D
tname('3D case');
dim = [3,3];
three_d_interp = general_interpolator(dim);
tspan3 = linspace(-1,5,12)';
dspan3 = rand([3, 3, length(tspan3)]);

three_d_interp.add_to_end_at_time(tspan3,dspan3);
% assert_near(three_d_interp.get_at_time(3), interp1(tspan2, dspan2, 3), tolerance, 'general_interpolator does not agree with interp1 in 3d case.');

% TODO TOMORROW.



end

