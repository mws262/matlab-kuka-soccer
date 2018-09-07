function test_make_ball()
tolerance = 1e-15;
theader('Testing make_ball.');

test_fig = figure('Visible', 'off');
radius = 5;
[ball_patch, ball_verts_untransformed] = make_ball(radius);

minZ = min(ball_patch.Vertices(:,3));
maxZ = max(ball_patch.Vertices(:,3));

minZ_init = min(ball_verts_untransformed(:,3));
maxZ_init = max(ball_verts_untransformed(:,3));

assert_near(maxZ, 2*radius, tolerance, 'Top of ball should be at 2x the radius above the ground.');
assert_near(minZ, 0, tolerance, 'Bottom of ball should be at the ground (0).');

assert_near(maxZ_init, 5, tolerance, 'Untransformed vertices should be min at -radius.');
assert_near(minZ_init, -5, tolerance, 'Untransformed vertices should be max at radius.');
close(test_fig);
end

