function save_plan_to_file(foldername, jnt_angle_cell, joint_timings, ...
    ball_timings, ball_com_positions, ball_com_velocities, ball_com_accelerations, ball_omegas, ball_quats, ...
    contact_pt_timings, contact_pt_positions, contact_pt_velocities, contact_status)
mkdir(foldername);

% TODO centralize stuff like this parameter.
ball_radius = 0.1;

% Unpack IK joint structures and save to a timestep x num_joints matrix
num_jnt_time_pts = length(jnt_angle_cell);
num_joints = length(jnt_angle_cell{1});
joints_over_time = zeros(num_jnt_time_pts, num_joints);
for i = 1:num_jnt_time_pts
    for j = 1:num_joints
        joints_over_time(i,j) = jnt_angle_cell{i}(j).JointPosition;
    end
end
if isrow(joint_timings)
    joint_timings = joint_timings';
end
if isrow(ball_timings)
    ball_timings = ball_timings';
end
if isrow(contact_pt_timings)
    contact_pt_timings = contact_pt_timings';
end

% [b,a] = butter(2,0.4); % lolpass filter
% joints_over_time_filtered = filtfilt(b,a,joints_over_time);


fit_discretization = length(joint_timings) * 10;
joint_angle_fit = pchip(joint_timings, joints_over_time');
joint_vel_fit = fnder(joint_angle_fit,1);
joint_accel_fit = fnder(joint_angle_fit,2);

joint_timings_fit = linspace(joint_timings(1), joint_timings(end), fit_discretization)';
joint_angles_fit_eval = ppval(joint_angle_fit, joint_timings_fit)';
joint_vel_fit_eval = ppval(joint_vel_fit, joint_timings_fit)';
joint_accel_fit_eval = ppval(joint_accel_fit, joint_timings_fit)';

% Revisit interpolation later.
% for i = 1e-1
% figure;
% subplot(2,1,1)
% p = spaps(joint_timings, joints_over_time', i)
% hold on;
% plot(joint_timings, joints_over_time, '.b', 'MarkerSize', 8);
% plot(fit_timings, fnval(p,fit_timings));
% xlabel(i);
% subplot(2,1,2);
% hold on;
% plot(fit_timings, fnval(fnder(p,1),fit_timings)/pi*180);
% legend();
% end

% Calculate ball angular accelerations
ball_alphas = [-ball_com_accelerations(:,2)/ball_radius, ball_com_accelerations(:,1)/ball_radius, zeros(size(ball_com_accelerations,1),1)];


save(['./', foldername, '/joint_timings_fit.mat'], 'joint_timings_fit', '-ascii', '-double');
save(['./', foldername, '/joint_angles_fit.mat'], 'joint_angles_fit_eval', '-ascii', '-double');
save(['./', foldername, '/joint_vel_fit.mat'], 'joint_vel_fit_eval', '-ascii', '-double');
save(['./', foldername, '/joint_accel_fit.mat'], 'joint_accel_fit_eval', '-ascii', '-double');

save(['./', foldername, '/joint_angles.mat'], 'joints_over_time', '-ascii', '-double');
% save(['./', foldername, '/joint_angles_filtered.mat'], 'joints_over_time_filtered', '-ascii', '-double');
save(['./', foldername, '/joint_timings.mat'], 'joint_timings', '-ascii', '-double');

save(['./', foldername, '/ball_timings.mat'], 'ball_timings', '-ascii', '-double');
save(['./', foldername, '/ball_com_positions.mat'], 'ball_com_positions', '-ascii', '-double');
save(['./', foldername, '/ball_com_velocities.mat'], 'ball_com_velocities', '-ascii', '-double');
save(['./', foldername, '/ball_com_accelerations.mat'], 'ball_com_accelerations', '-ascii', '-double');

save(['./', foldername, '/ball_omegas.mat'], 'ball_omegas', '-ascii', '-double');
save(['./', foldername, '/ball_alphas.mat'], 'ball_alphas', '-ascii', '-double');
save(['./', foldername, '/ball_quats.mat'], 'ball_quats', '-ascii', '-double');

save(['./', foldername, '/contact_pt_timings.mat'], 'contact_pt_timings', '-ascii', '-double');
save(['./', foldername, '/contact_pt_positions.mat'], 'contact_pt_positions', '-ascii', '-double');
save(['./', foldername, '/contact_pt_velocities.mat'], 'contact_pt_velocities', '-ascii', '-double');

% save doesn't like to write logic values, so we're going to do it by hand
% here.
fcontact = fopen(['./', foldername, '/contact_status.mat'], 'w');
for i = 1:length(contact_status)
   fprintf(fcontact, '%i\n',contact_status(i)); 
end
fclose(fcontact);

fid = fopen(['./', foldername, '/notes.txt'], 'w');
fprintf(fid, 'joint_angles have timings on the rows, different joints on the columns, going from 1->7.\n');
fprintf(fid, 'joint_angles have rows matching joint_timings. All ball data matches the rows in ball_timings.\n');
fprintf(fid, 'contact_pt_timings cover the same time range as ball timings, but with gaps, since the contact point is intermittent.\n');
fprintf(fid, 'Unless otherwise noted, ball radius is 0.1m.\n');
fprintf(fid, 'joint_fit... are for pchip interpolated and fnder-differentiated joint angle values.\n');
fclose(fid);

zip([foldername, '.zip'], ['./', foldername, '/']);
end

