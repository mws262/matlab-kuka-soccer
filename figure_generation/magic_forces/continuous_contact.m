close all; clear all;
addpath ../../;
addpath ../../vis/;
addpath ../../geometry/;
addpath ../../derived_autogen/;
addpath ../../data/;
addpath ../../dynamics/;
addpath ../../util/;

vid_dir = '../../data/videos/raw/';
%%%%%%%%%%%% FIGURE 8 %%%%%%%%%%%%%%%%%%%%
%% Shape parameters
segs_between = 3; % One of the most important parameters. Will change the number of extra segments between required knot points.
time_scaling = 5; % Time to make one cycle along the 8.
lobe_length = 0.5; % Parameters to stretch the 8.
lobe_width = 0.2;
lobe_center_offset = 0;
offset = [1,1,0];
scale = 1;
height = 0; % Height of path off the ground. I see no reason to change this.

%% Knots and breaks
knots = [0, 0, height;
    lobe_width, lobe_length/2 + lobe_center_offset, height;
    0, lobe_length, height;
    -lobe_width, lobe_length/2 + lobe_center_offset, height
    0, 0, height;
    lobe_width, -lobe_length/2 - lobe_center_offset, height;
    0, -lobe_length, height;
    -lobe_width, -lobe_length/2 - lobe_center_offset, height;
    0, 0, height]';

num_knots = size(knots,2);
breaks = linspace(0, time_scaling, num_knots);

%% QP version
qp_sol = qp_spline(breaks, knots', segs_between);

%% Ball and scene
ball_radius = 0.1;
scene_fig = make_visualizer_scene();
[ball_patch, ball_verts_untransformed] = make_ball(ball_radius);

num_pts = 250;

[tspan, posspan, velspan, accelspan, omegaspan, quatspan, ...
    world_contact_loc_desired_fcn, contact_loc_desired_rel_com_fcn] = evaluate_spline(qp_sol, ball_radius, num_pts);

[path_line, accel_arrows] = draw_path_and_accel(posspan, accelspan, 3);

%% Play the winning solution.
campos([    0.5748   -0.0755    1.0766]);
tfactor = 1;
tic;

% Video recording if desired
num_laps = 5;
write_to_vid = true;
if write_to_vid
    framerate = 60;
    vid_writer = VideoWriter([vid_dir, 'magic_force_continuous_contact_periodic.avi']); % Convert for Slack with 'ffmpeg -i infile.avi youroutput.mp4'
    vid_writer.FrameRate = framerate;
    vid_writer.Quality = 100;
    open(vid_writer);
end

laps_complete = 0;
curr_time = 0;
prev_time = 0;
while ishandle(scene_fig)
    
    quat_eval = interp1(tspan, quatspan, curr_time);
    pos_eval = interp1(tspan, posspan, curr_time);
    ball_patch.Vertices = quatrotate(quat_eval, ball_verts_untransformed) + repmat(pos_eval  + [0, 0, ball_radius], [size(ball_verts_untransformed,1),1]);
    
    drawnow;

    if write_to_vid
        writeVideo(vid_writer, getframe(scene_fig));
        curr_time = 1./framerate * tfactor + curr_time;      
    else
       curr_time = toc * tfactor;
    end
       curr_time = mod(curr_time, tspan(end)); 
       if prev_time > curr_time
          laps_complete = laps_complete + 1;
       end
       if laps_complete == num_laps
           break;
       end 
       prev_time = curr_time;
end

if write_to_vid
    close(vid_writer);
end
