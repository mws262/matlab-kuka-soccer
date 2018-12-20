contact_pt = load('contact_pt_positions.mat', '-ascii');

ball_pos = load('ball_com_positions.mat', '-ascii');
ball_vel = load('ball_com_velocities.mat', '-ascii');
ball_omega = load('ball_omegas.mat', '-ascii');

box_pos = load('box_positions.mat', '-ascii');
box_vel = load('box_linear_vel.mat', '-ascii');
box_omega = load('box_angular_vel.mat', '-ascii');

% Contact point velocity according to the ball.
ctact_vel_ball = ball_vel + cross(ball_omega, contact_pt - ball_pos);

% Contact point velocity according to the box.
ctact_vel_box = box_vel + cross(box_omega, contact_pt - box_pos);

vel_diff = ctact_vel_ball - ctact_vel_box;

disp(max(abs(vel_diff(:))));
