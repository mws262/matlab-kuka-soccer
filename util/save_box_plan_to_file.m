function save_box_plan_to_file(foldername, timings, box_positions, box_quats, box_linear_vel, box_angular_vel, ...
    ball_com_positions, ball_com_velocities, ball_com_accelerations, ball_quats, ball_omegas, ...
    contact_pt_positions, contact_pt_velocities, contact_status, notes)
mkdir(foldername);

% TODO centralize stuff like this parameter.
ball_radius = 0.1;

if isrow(timings)
    timings = timings';
end

% [b,a] = butter(2,0.4); % lolpass filter
% joints_over_time_filtered = filtfilt(b,a,joints_over_time);


% Calculate ball angular accelerations
ball_alphas = [-ball_com_accelerations(:,2)/ball_radius, ball_com_accelerations(:,1)/ball_radius, zeros(size(ball_com_accelerations,1),1)];

save(['./', foldername, '/timings.mat'], 'timings', '-ascii', '-double');
save(['./', foldername, '/ball_com_positions.mat'], 'ball_com_positions', '-ascii', '-double');
save(['./', foldername, '/ball_com_velocities.mat'], 'ball_com_velocities', '-ascii', '-double');
save(['./', foldername, '/ball_com_accelerations.mat'], 'ball_com_accelerations', '-ascii', '-double');

save(['./', foldername, '/ball_omegas.mat'], 'ball_omegas', '-ascii', '-double');
save(['./', foldername, '/ball_alphas.mat'], 'ball_alphas', '-ascii', '-double');
save(['./', foldername, '/ball_quats.mat'], 'ball_quats', '-ascii', '-double');

save(['./', foldername, '/contact_pt_positions.mat'], 'contact_pt_positions', '-ascii', '-double');
save(['./', foldername, '/contact_pt_velocities.mat'], 'contact_pt_velocities', '-ascii', '-double');

save(['./', foldername, '/box_positions.mat'], 'box_positions', '-ascii', '-double');
save(['./', foldername, '/box_quats.mat'], 'box_quats', '-ascii', '-double');
save(['./', foldername, '/box_linear_vel.mat'], 'box_linear_vel', '-ascii', '-double');
save(['./', foldername, '/box_angular_vel.mat'], 'box_angular_vel', '-ascii', '-double');

% save doesn't like to write logic values, so we're going to do it by hand
% here.
fcontact = fopen(['./', foldername, '/contact_status.mat'], 'w');
for i = 1:length(contact_status)
   fprintf(fcontact, '%i\n',contact_status(i)); 
end
fclose(fcontact);

fid = fopen(['./', foldername, '/notes.txt'], 'w');
fprintf(fid, strcat(notes, '\n'));
% fprintf(fid, 'joint_angles have timings on the rows, different joints on the columns, going from 1->7.\n');
% fprintf(fid, 'joint_angles have rows matching joint_timings. All ball data matches the rows in ball_timings.\n');
% fprintf(fid, 'contact_pt_timings cover the same time range as ball timings, but with gaps, since the contact point is intermittent.\n');
% fprintf(fid, 'Unless otherwise noted, ball radius is 0.1m.\n');
% fprintf(fid, 'joint_fit... are for pchip interpolated and fnder-differentiated joint angle values.\n');
fclose(fid);

zip([foldername, '.zip'], ['./', foldername, '/']);
end

