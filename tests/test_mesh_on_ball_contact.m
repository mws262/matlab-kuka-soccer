close all; clear all;
addpath ../;
addpath ../vis/;
addpath ../geometry/;
addpath ../derived_autogen/;
addpath ../data/;

% Ball and scene
radius = 0.1;
ball_velocity = [0, -1, 0];
scene_fig = make_visualizer_scene();
[ball_patch, ball_verts_untransformed] = make_ball(radius);

% Manipulator mesh
detail_level = 1;
geo_data = load('../data/iiwa_merged_end_effector.mat');
picked_pts = load('../data/picked_faces.mat', 'selections');
faces = geo_data.merged_iiwa(detail_level).faces;
vertices = geo_data.merged_iiwa(detail_level).vertices;
face_normals = geo_data.merged_iiwa(detail_level).face_normals;
vertex_normals = geo_data.merged_iiwa(detail_level).vertex_normals;
iiwa_patch = patch('Faces', faces, 'Vertices', vertices, 'FaceNormals', face_normals, 'VertexNormals', vertex_normals);
iiwa_patch.EdgeColor = [0.1, 0.1, 0.1];
iiwa_patch.FaceColor = [1 0 0];
iiwa_patch.FaceAlpha = 0.85;
iiwa_tform = hgtransform;
iiwa_patch.Parent = iiwa_tform;

% States, etc
tspan = linspace(0,5,600)';
zero_vec_pts = zeros(size(tspan));
lin_vel = repmat(ball_velocity, [length(tspan), 1]); % Constant velocity for this.
pos = cumtrapz(tspan, lin_vel);
omega = [angular_rate_wx_fcn(radius, lin_vel(:,2)), angular_rate_wy_fcn(radius, lin_vel(:,1)), zero_vec_pts];

rotationQ = [1 0 0 0]; % Quaternion representing ball rotation.
contact_loc = repmat([0 0 2*radius],[length(tspan), 1]);
contact_loc_over_time = contact_loc + pos;
surface_vel = cross(omega, contact_loc, 2);
% These go singular when accel is 0.
% v_surfx_eval = v_surfx_fcn(zero_vec_pts, zero_vec_pts, pi/2 - 0.1, lin_vel(:,1), lin_vel(:,2));
% v_surfy_eval = v_surfy_fcn(zero_vec_pts, zero_vec_pts, pi/2 - 0.1, lin_vel(:,1), lin_vel(:,2));

picked_idx = 3;
initial_surface_point = picked_pts.selections.points(picked_idx);
% [ distance, surf_target, surf_normal ] = point2trimesh_with_normals( initial_surface_point, faces, vertices, face_normals, vertex_normals );
[result_path_pts, result_path_normals, rot_integrated] = integrate_velocity_over_surface(tspan, surface_vel, initial_surface_point, [0 0 1], geo_data.merged_iiwa(detail_level));

% Video recording if desired
write_to_vid = false;
if write_to_vid
    framerate = 30;
    vid_writer = VideoWriter('rolly_link_vid2.avi'); % Convert for Slack with 'ffmpeg -i infile.avi youroutput.mp4'
    vid_writer.FrameRate = framerate;
    vid_writer.Quality = 100;
    open(vid_writer);
end
for i = 1:length(tspan) - 1
    dt = tspan(i+1) - tspan(i);
    rotationQ = 0.5 * quatprod([0, omega(i,:)], rotationQ) * dt + rotationQ;
    rotationQ = rotationQ/norm(rotationQ);
    reverseQ = rotationQ .* [-1 1 1 1]; % MATLAB's reverse right-hand-rule for quaternions. Wow!
    ball_patch.Vertices = quatrotate(reverseQ, ball_verts_untransformed) + repmat(pos(i,:)  + [0, 0, radius], [size(ball_verts_untransformed,1),1]);
    
    surface_to_origin_translation = trvec2tform(-result_path_pts(i,:));
    world_origin_to_goal_pt_translation = trvec2tform(contact_loc_over_time(i,:));
    
    surf_vel_untransformed_to_planar_vel = rotm2tform(rot_integrated(:,:,i)');
    
    iiwa_tform.Matrix = world_origin_to_goal_pt_translation * surf_vel_untransformed_to_planar_vel * surface_to_origin_translation;
    
    camvec = (angle2dcm(0,0,i/200, 'xyz')*(lin_vel(i,:)*1.2 + [0.0 0 .25])')';
    camtarget(pos(i,:));
    campos(pos(i,:) + camvec);
    drawnow;
    %pause(0.001);
    if write_to_vid
        writeVideo(vid_writer, getframe(scene_fig));
    end
end

if write_to_vid
    close(vid_writer);
    % Convert from avi to mp4.
    !ffmpeg -i make_8_vid.avi make_8_vid.mp4
end