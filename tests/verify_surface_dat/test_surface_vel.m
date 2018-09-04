clear all; close all;
folder = './small_arc_dat/';

% On ball timings.
ball_com_velocities = load([folder, 'ball_com_velocities.mat'], '-ascii');
ball_timings = load([folder, 'ball_timings.mat'], '-ascii');
ball_com_positions = load([folder, 'ball_com_positions.mat'], '-ascii');

% On contact timings.
contact_pt = load([folder, 'contact_pt_positions.mat'], '-ascii');
contact_pt_timings = load([folder, 'contact_pt_timings.mat'], '-ascii');
contact_velocities_rel_com = load([folder, 'contact_pt_velocities.mat'], '-ascii');

% On joint timings.
joint_timings = load([folder, 'joint_timings.mat'], '-ascii');
joint_angles = load([folder, 'joint_angles.mat'], '-ascii');

% On joint fit timings.
joint_fit_timings = load([folder, 'joint_timings_fit.mat'], '-ascii');
joint_angle_fit = load([folder, 'joint_angles_fit.mat'], '-ascii');

iiwa = IIWAImporter(figure);
template_config_struct = iiwa.home_config;
all_joint_angles = cell(1, size(joint_angle_fit,1));

% Pack the fit joint angles back into struct form for ik.
for j = 1:size(joint_angle_fit,1)
    for i = 1:length(template_config_struct)
        template_config_struct(i).JointPosition = joint_angle_fit(j,i);
        all_joint_angles{j} = template_config_struct;
    end
end

% Joint angles just during contact.
jnt_angles_ctact = interp1(joint_fit_timings, joint_angle_fit, contact_pt_timings, 'linear', 'extrap');
jnt_struct_ctact = cell(1, size(contact_pt_timings,1));

for j = 1:size(jnt_angles_ctact,1)
    for i = 1:length(template_config_struct)
        template_config_struct(i).JointPosition = jnt_angles_ctact(j,i);
        jnt_struct_ctact{j} = template_config_struct;
    end
end

% Compute link 6 frame vel
joint_vel_fit = load([folder, 'joint_vel_fit.mat'], '-ascii');
est_link_vel = zeros(6,size(joint_vel_fit,1));
for i = 1:size(joint_vel_fit,1)
    est_link_vel(:,i) = geometricJacobian(iiwa.robot, all_joint_angles{i}, 'iiwa_link_6') * joint_vel_fit(i,:)';
end


% Transform contact path on dummy link to coordinates of iiwa.
total_dummy_to_robot = [0    1.0000         0         0
         0         0   -1.0000         0
   -1.0000         0         0    0.2155
         0         0         0    1.0000];
     
dummy_to_robot_rotation = tform2rotm(total_dummy_to_robot);
dummy_to_robot_translation = tform2trvec(total_dummy_to_robot);

% Transform path points on dummy to link's so that
% getTransform(...)*path_pts_robot matches the path in the real world.

robot_to_dummy = rotm2tform(dummy_to_robot_rotation')*trvec2tform(-dummy_to_robot_translation);
path_pts_robot = robot_to_dummy*[contact_pt'; ones(1,size(contact_pt,1))];

% % Positions
% l6tforms = zeros(4,4,length(jnt_struct_ctact));
% spatial_arm_pts = zeros(length(jnt_struct_ctact),4);
% for i = 1:length(jnt_struct_ctact)
%     l6tforms(:,:,i) = getTransform(iiwa.robot, jnt_struct_ctact{i}, 'iiwa_link_6','iiwa_link_0');
%     spatial_arm_pts(i,:) = l6tforms(:,:,i) * path_pts_robot(:,i);
% end



contact_normal = (contact_pt - interp1(ball_timings, ball_com_positions, contact_pt_timings))/0.1;
planar_tforms = zeros(3,3,size(contact_normal,1));
for i = 1:size(contact_normal,1)
    planar_tforms(:,:,i) = get_rotation_from_vecs(contact_normal(i,:), [0,0,1]);
end

% Interpolate link com velocities to the contact timings. Separated angular
% and linear rates.
lin_vel_link_at_contact_times = interp1(joint_fit_timings, est_link_vel(4:end,:)', contact_pt_timings);
ang_vel_link_at_contact_times = interp1(joint_fit_timings, est_link_vel(1:3,:)', contact_pt_timings);

arm_pt_vel = cross(ang_vel_link_at_contact_times, path_pts_robot(1:3,:)') + lin_vel_link_at_contact_times; % Vel at surface point of the arm.

% total_contact_pt_vel_ball = contact_velocities_rel_com + interp1(ball_timings, ball_com_velocities,contact_pt_timings); % Ve

ball_pt_vel = contact_velocities_rel_com; %+ interp1(ball_timings, ball_com_velocities, contact_pt_timings); %Already took linear term into account

tformed_ball_pt_vel = zeros(size(ball_pt_vel));
tformed_arm_pt_vel = zeros(size(ball_pt_vel));

for i = 1:size(arm_pt_vel,1)
   tformed_ball_pt_vel(i,:) = (planar_tforms(:,:,i) * ball_pt_vel(i,:)')'; 
   tformed_arm_pt_vel(i,:) = (planar_tforms(:,:,i) * arm_pt_vel(i,:)')';
end

figure;

for i = 1:3
    subplot(3,1,i);
    hold on;
    plot(contact_pt_timings, tformed_arm_pt_vel(:,i), 'r.');
    plot(contact_pt_timings, tformed_ball_pt_vel(:,i), 'b.');
end






