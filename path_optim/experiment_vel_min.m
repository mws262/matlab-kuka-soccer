close all; clear all;

input_data = load('test_traj.mat', 'joints_over_time', 'ball_pt_vel', 'est_joint_times', 'iiwa');

data_x = input_data.joints_over_time;

data_t = input_data.est_joint_times;

eval_pts = linspace(0,1,size(data_x,1) * 10);

basic_spline = spline(data_t, data_x');
basic_spline_dt = fnder(basic_spline,1);

pos = ppval(basic_spline, eval_pts);
vel = ppval(basic_spline_dt, eval_pts);

figure;
subplot(2,1,1);
hold on;
plot(eval_pts, pos);
plot(data_t, data_x, '.');
hold off;
subplot(2,1,2);
plot(eval_pts,vel);

%%%%%%%%%%%%%%%%%%%%%%%





