% Test mesh in contact with the ball as the ball rolls along a spline path.
% The contact location on the ball should obey the acceleration
% requirements. This is mostly to make sure that the geometry of the
% rolling contact is correct.

close all; clear all;
addpath ../;
addpath ../vis/;
addpath ../geometry/;
addpath ../derived_autogen/;
addpath ../data/;

% Ball and scene
ball_radius = 0.1;
scene_fig = make_visualizer_scene();
[ball_patch, ball_verts_untransformed] = make_ball(ball_radius);

% Manipulator mesh
detail_level = 1;
picked_idx = 1;
debug_mesh = false; % Just load some convex shape instead of a root part.
% picked_pts = load('../data/picked_faces_single_end.mat', 'selections');
if debug_mesh
   mesh_data = get_mesh_data('horiz_plane');
   initial_surface_point = [0,0.1,0.15]; % Note that the loaded one is in a bad spot. I think it is inside this mesh, which inverts the problem a tad.
   cmap = flag(size(mesh_data.faces,1));
else
    mesh_data = get_mesh_data('dummy_manipulator_high_res');
    banned_region = get_mesh_data('manipulator_banned1');
    cmap = repmat([0 0 1], [size(mesh_data.faces,1),1]);
end

faces = mesh_data.faces;
vertices = mesh_data.vertices;
face_normals = mesh_data.face_normals;
vertex_normals = mesh_data.vertex_normals;

manipulator_patch = patch('Faces', faces, 'Vertices', vertices, 'FaceNormals', face_normals, 'VertexNormals', vertex_normals);
manipulator_patch.EdgeColor = [0.1, 0.1, 0.1];
manipulator_patch.FaceColor = 'flat';
manipulator_patch.FaceVertexCData = cmap;
manipulator_patch.FaceAlpha = 0.65;
manipulator_patch.BackFaceLighting = 'lit';
manipulator_tform = hgtransform;
manipulator_patch.Parent = manipulator_tform;


%% Pick a path polynomial.
path_pp = get_path_pp('triangle', 5);

total_ts = 250; % Total timesteps to evaluate at.
approach_ang = 0;
arc_angle = pi/4; % Angle along the possible arc of the ball to contact.
initial_surface_point = [0,0.1,0.3]; % Initial point on the surface to project down.

%% Evaluate ball and contact point quantities.
[tspan, posspan, velspan, accelspan, omegaspan, quatspan, ...
    world_contact_loc_desired_fcn, contact_loc_desired_rel_com_fcn] = evaluate_spline(path_pp, ball_radius, total_ts);

world_contact_desired_span = world_contact_loc_desired_fcn(arc_angle*ones(size(tspan)));
contact_desired_rel_com_span = contact_loc_desired_rel_com_fcn(arc_angle*ones(size(tspan)));

up_vector_span = contact_desired_rel_com_span/ball_radius;
surface_vel_span = cross(omegaspan, contact_desired_rel_com_span, 2);

[result_path_pts, result_path_normals, rot_integrated] = integrate_velocity_over_surface(tspan, surface_vel_span, initial_surface_point, ...
    up_vector_span, approach_ang, mesh_data);%, banned_region);

%% Static plot things and 
hold on;
draw_path_and_accel(posspan, accelspan, 3);

omega_pl = quiver3(0,0,0,0,0,0);
surface_vel_pl = quiver3(0,0,0,0,0,0);
contact_pt_pl = plot(0,0, '.g', 'MarkerSize', 20);

% Video recording if desired
write_to_vid = false;
if write_to_vid
    framerate = 60;
    vid_writer = VideoWriter('rolly_link_vid2.avi'); % Convert for Slack with 'ffmpeg -i infile.avi youroutput.mp4'
    vid_writer.FrameRate = framerate;
    vid_writer.Quality = 100;
    open(vid_writer);
end

% Still sorting this mess out.
% upvec_rot = zeros(3,3,length(tspan));
% upvec_rot(:,:,1) = eye(3);
% for i = 2:length(tspan)
% next_rot = get_rotation_from_vecs(up_vector_span(i - 1, :), up_vector_span(i, :));
% upvec_rot(:,:,i) = next_rot * upvec_rot(:,:,i-1);
% end


for i = 1:length(tspan) - 1
    quat = quatspan(i,:);
    ball_patch.Vertices = quatrotate(quat, ball_verts_untransformed) + repmat(posspan(i,:)  + [0, 0, ball_radius], [size(ball_verts_untransformed,1),1]);
    
    surface_to_origin_translation = trvec2tform(-result_path_pts(i,:));
    world_origin_to_goal_pt_translation = trvec2tform(world_contact_desired_span(i,:));
    
    surf_vel_untransformed_to_planar_vel = rotm2tform(rot_integrated(:,:,i)');
    ball_contact_shift_tform = rotm2tform(get_rotation_from_vecs(up_vector_span(1,:), up_vector_span(i,:))); % upvec_rot(:,:,i));%
    manipulator_tform.Matrix = world_origin_to_goal_pt_translation * ball_contact_shift_tform * surf_vel_untransformed_to_planar_vel * surface_to_origin_translation;
    
    % Show contact point.
    contact_pt_pl.XData = world_contact_desired_span(i,1);
    contact_pt_pl.YData = world_contact_desired_span(i,2);
    contact_pt_pl.ZData = world_contact_desired_span(i,3);
    
    % Surface velocity arrow.
    surface_vel_pl.XData = world_contact_desired_span(i,1);
    surface_vel_pl.YData = world_contact_desired_span(i,2);
    surface_vel_pl.ZData = world_contact_desired_span(i,3);
    surface_vel_pl.UData = surface_vel_span(i,1) + velspan(i,1);
    surface_vel_pl.VData = surface_vel_span(i,2) + velspan(i,2);
    surface_vel_pl.WData = surface_vel_span(i,3) + velspan(i,3);
    
    % Ball rotation axis arrow.
    omega_pl.XData = posspan(i,1);
    omega_pl.YData = posspan(i,2);
    omega_pl.ZData = posspan(i,3) + ball_radius;
    omega_pl.UData = omegaspan(i,1);
    omega_pl.VData = omegaspan(i,2);
    omega_pl.WData = omegaspan(i,3);
    drawnow;
    pause(0.05);
    if write_to_vid
        writeVideo(vid_writer, getframe(scene_fig));
    end
end

if write_to_vid
    close(vid_writer);
    % Convert from avi to mp4.
    !ffmpeg -i make_8_vid.avi make_8_vid.mp4
end