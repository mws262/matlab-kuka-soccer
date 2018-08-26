function playback_plan(directory_name, playback_time_factor, write_to_vid)
% Give it a directory created by save_plan_to_file.m

ball_com_positions = load([directory_name, '/ball_com_positions.mat'], 'ball_com_positions', '-ascii');
% ball_com_velocities = load([directory_name, '/ball_com_velocities.mat'], 'ball_com_velocities', '-ascii');
ball_com_accelerations = load([ directory_name, '/ball_com_accelerations.mat'], 'ball_com_accelerations', '-ascii');
% ball_com_omegas = load([directory_name, '/ball_omegas.mat'], 'ball_omegas', '-ascii');
ball_quats = load([directory_name, '/ball_quats.mat'], 'ball_quats', '-ascii');
ball_timings = load([directory_name, '/ball_timings.mat'], 'ball_timings', '-ascii');

joint_timings = load([directory_name, '/joint_fit_timings.mat'], 'joint_timings', '-ascii');
joint_angles = load([directory_name, '/joint_angles_fit.mat'], 'joint_angles', '-ascii');

% contact_pt_timings = load([directory_name, '/contact_pt_timings.mat'], 'contact_pt_timings', '-ascii');
% contact_pt_positions = load([directory_name, '/contact_pt_positions.mat'], 'contact_pt_positions', '-ascii');

%% Set up world scene.
% World scene.
scene_fig = make_visualizer_scene();

% Ball.
hold on;
ball_radius = 0.1;
[ball_patch, ball_verts_untransformed] = make_ball(ball_radius);

% Whole robot.
iiwa = IIWAImporter(scene_fig);

draw_path_and_accel(ball_com_positions - [0, 0, ball_radius], ball_com_accelerations, 1); % Draw out the path and acceleration arrows.

template_config_struct = iiwa.home_config;
all_joint_angles = cell(1, size(joint_angles,1));
for j = 1:size(joint_angles,1)
    for i = 1:length(template_config_struct)
        template_config_struct(i).JointPosition = joint_angles(j,i);
        all_joint_angles{j} = template_config_struct;
    end
end

%% Play the winning solution.
campos([1.6970    1.5293    0.9466]);
tic;
curr_time = 0;

% Video recording if desired
write_to_vid = false;
if write_to_vid
    framerate = 60;
    vid_writer = VideoWriter([directory_name, '.avi']); % Convert for Slack with 'ffmpeg -i infile.avi youroutput.mp4'
    vid_writer.FrameRate = framerate;
    vid_writer.Quality = 100;
    open(vid_writer);
end

while ishandle(scene_fig)
    
    quat_eval = interp1(ball_timings, ball_quats, curr_time);
    pos_eval = interp1(ball_timings, ball_com_positions, curr_time);
    ball_patch.Vertices = quatrotate(quat_eval, ball_verts_untransformed) + repmat(pos_eval, [size(ball_verts_untransformed,1),1]);
    
    joint_pos_struct = interpolate_traj(iiwa, joint_timings, all_joint_angles, curr_time);
    display_at_pose(iiwa, joint_pos_struct);
    
    drawnow;
    
    if write_to_vid
        writeVideo(vid_writer, getframe(scene_fig));
        curr_time = 1./framerate * playback_time_factor + curr_time;
    else
        curr_time = toc * playback_time_factor;
    end
    curr_time = mod(curr_time, ball_timings(end));
end
if write_to_vid
    close(vid_writer);
end
end
