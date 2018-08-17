function test_mesh_on_ball_w_ik
close all; clear all;
addpath ../;
addpath ../vis/;
addpath ../geometry/;
addpath ../derived_autogen/;
addpath ../data/;

% This transforms the graphical version of the free-floating foot for
% planning onto the graphical version of the foot on the full robot. This
% was partially hand-tuned.
% Note that there is another transform which goes between the real version
% of the link in the RigidBodyTree and the graphical version.
tform_dummy_to_robot =    [ 0.0000    0.0000    1.0000   -0.2155;
    0.0000   -1.0000    0.0000    0.0000;
    1.0000    0.0000   -0.0000    1.1800;
    0         0         0    1.0000]
inv_tform_dummy_to_robot = inv(tform_dummy_to_robot);

% Ball and scene
radius = 0.1; %BALL radius
ball_velocity = [0, -1, 0];
scene_fig = make_visualizer_scene();
[ball_patch, ball_verts_untransformed] = make_ball(radius);
hold on;
% Manipulator mesh
detail_level = 1;
geo_data = load('../data/iiwa_merged_end_effector.mat');
picked_pts = load('../data/picked_faces_single_end.mat', 'selections');
faces = geo_data.merged_iiwa(detail_level).faces;
vertices = geo_data.merged_iiwa(detail_level).vertices;
face_normals = geo_data.merged_iiwa(detail_level).face_normals;
vertex_normals = geo_data.merged_iiwa(detail_level).vertex_normals;
iiwa_patch = patch('Faces', faces, 'Vertices', vertices, 'FaceNormals', face_normals, 'VertexNormals', vertex_normals);
iiwa_patch.EdgeColor = 'none';%[0.1, 0.1, 0.1];
iiwa_patch.FaceColor = 'flat';
iiwa_patch.FaceVertexCData = repmat([0 0 1], [size(iiwa_patch.Faces,1),1]);
iiwa_patch.FaceAlpha = 0.8;
iiwa_patch.BackFaceLighting = 'lit';
iiwa_tform = hgtransform;
iiwa_patch.Parent = iiwa_tform;

% Whole robot.

iiwa = IIWAImporter(scene_fig);
link_name = 'iiwa_link_6'; % Tip of ee. iiwa_link_ee is just the last full link.
home = iiwa.home_config;
guess = home;
tform_init = getIIWATForm(iiwa, home, link_name);
% iiwa.link_patches{2}.FaceAlpha = 0.5
% iiwa.link_patches{1}.FaceAlpha = 0.5

link6transform = iiwa.hgtransforms{2};

% More interesting path.
tend = 5;
laps = 1;
R = 0.04; % path radius
offset = [0.6;0;0];
% knots = [0,0,0; R,R,0; 0,2*R,0; -R,R,0;0,0,0]' + offset;
knots = [0,0,0; 2*R,0,0;-R,0,0; 0,0,0]' + offset;
breaks = linspace(0, tend, size(knots,2));

pos_pp = csape(breaks, knots, 'periodic');
vel_pp = fnder(pos_pp);
accel_pp = fnder(vel_pp);
pts_per_lap = 250;
tspan_eval = linspace(0, tend, pts_per_lap)';
pos_eval = repmat(ppval(pos_pp, tspan_eval)',[laps,1]);
vel_eval = repmat(ppval(vel_pp, tspan_eval)',[laps,1]);
accel_eval = repmat(ppval(accel_pp, tspan_eval)',[laps,1]);
tspan = linspace(0, tend*laps, pts_per_lap*laps)';
hold on;
plot(pos_eval(:,1), pos_eval(:,2), 'LineWidth', 2);


% States, etc
% tspan = linspace(0,5,600)';
zero_vec_pts = zeros(size(tspan));
pos = cumtrapz(tspan, vel_eval) + offset';
omega = [angular_rate_wx_fcn(radius, vel_eval(:,2)), angular_rate_wy_fcn(radius, vel_eval(:,1)), zero_vec_pts];

rotationQ = [1 0 0 0]; % Quaternion representing ball rotation.
contact_loc = repmat([0 0 2*radius],[length(tspan), 1]);
contact_loc_over_time = contact_loc + pos;
surface_vel = cross(omega, contact_loc, 2);
% These go singular when accel is 0.
% v_surfx_eval = v_surfx_fcn(zero_vec_pts, zero_vec_pts, pi/2 - 0.1, lin_vel(:,1), lin_vel(:,2));
% v_surfy_eval = v_surfy_fcn(zero_vec_pts, zero_vec_pts, pi/2 - 0.1, lin_vel(:,1), lin_vel(:,2));


ball_patch.Vertices = ball_verts_untransformed + repmat(pos(1,:)  + [0, 0, radius], [size(ball_verts_untransformed,1),1]);
iiwa_patch.Visible = 'off';
%%%
angles = linspace(0,2*pi,20);
sample_faces = length(picked_pts.selections.faces);
res = zeros(length(angles), length(sample_faces));
% for k = 1:sample_faces
%     tic
%     
%     for i = 1:length(angles)
%         
%         initial_surface_point = picked_pts.selections.points(k,:);
%         [rot, current_pt, current_normal] = find_mesh_contact_tform(initial_surface_point, [0,0,1], angles(i), iiwa_patch.Faces, iiwa_patch.Vertices, iiwa_patch.FaceNormals, iiwa_patch.VertexNormals);
%         tform = trvec2tform(contact_loc_over_time(1,:))*(rotm2tform(rot))*trvec2tform(-current_pt)*inv_tform_dummy_to_robot*getTransform(iiwa.robot, home, link_name);
%         
%         [jnt_angle, solinfo] = single_ik_call(iiwa, tform, guess, link_name);
%         res(i,k) = solinfo.PoseErrorNorm;
%         % disp(solinfo.Status);
%         %     display_at_pose(iiwa, jnt_angle);
%         %     drawnow;
%     end
%     toc
% end

% maxx = min(iiwa_patch.Vertices(:,1),[],1);
% minx = max(iiwa_patch.Vertices(:,1),[],1);
% maxy = min(iiwa_patch.Vertices(:,2),[],1);
% miny = max(iiwa_patch.Vertices(:,2),[],1);
% maxz = min(iiwa_patch.Vertices(:,3),[],1);
% minz = max(iiwa_patch.Vertices(:,3),[],1);
% % longitudinal axis aligned with x.
% cyl_rad = max(maxy - miny, maxz - minz)/2;
% cyl_off_y = (miny + maxy)/2;
% cyl_off_z = (minz + maxz)/2;
% cyl_off_x = minx;
% cyl_height = maxx - minx;
% 
% [xres, fval] = cmaes('cost_fun_wrap', [ cyl_off_x + cyl_height/2, 0, pi/2], 0.1)
% 
%     function err = cost_fun_wrap(X)
%         err = cost_fun(X(:,1), X(:,2), X(:,3));
%     end
%     function err = cost_fun(proj_height, proj_ang, placement_ang)
%         init_s_pt = [proj_height, cyl_rad*cos(proj_ang), cyl_rad*sin(proj_ang)];
%         
%         [rot, current_pt, current_normal] = find_mesh_contact_tform(init_s_pt, [0,0,1], placement_ang, iiwa_patch.Faces, iiwa_patch.Vertices, iiwa_patch.FaceNormals, iiwa_patch.VertexNormals);
%         tform = trvec2tform(contact_loc_over_time(1,:))*(rotm2tform(rot))*trvec2tform(-current_pt)*inv_tform_dummy_to_robot*getTransform(iiwa.robot, home, link_name);
%         
%         [jnt_angle, solinfo] = single_ik_call(iiwa, tform, guess, link_name);
%         err = solinfo.PoseErrorNorm;
%     end
% 
% 
% %%%%
% angles = linspace(0,2*pi,20);
% ik_idx = floor(linspace(1,length(tspan) - 1, 4));
% iiwa_breaks = tspan(ik_idx);
% for i = 1:length(picked_pts.selections.faces)
%     for j = 1:20
%         initial_surface_point = picked_pts.selections.points(i,:);
%         [result_path_pts, result_path_normals, rot_integrated] = integrate_velocity_over_surface(tspan, surface_vel, initial_surface_point, [0 0 1], angles(j), geo_data.merged_iiwa(detail_level));
%         surface_to_origin_translation = trvec2tform(-result_path_pts(ik_idx,:));
%         world_origin_to_goal_pt_translation = trvec2tform(contact_loc_over_time(ik_idx,:));
%         surf_vel_untransformed_to_planar_vel = rotm2tform(permute(rot_integrated(:,:,ik_idx), [2,1,3]));
%         iiwa_knots = zeros(4,4,size(surf_vel_untransformed_to_planar_vel,3));
%         for k = 1:size(surf_vel_untransformed_to_planar_vel,3)
%             iiwa_knots(:,:,k) = world_origin_to_goal_pt_translation(:,:,k) * surf_vel_untransformed_to_planar_vel(:,:,k) * surface_to_origin_translation(:,:,k)*inv_tform_dummy_to_robot*getTransform(iiwa.robot, home, link_name);
%         end
%         [jnt_angles, solinfo] = make_multi_point_trajectory(iiwa, iiwa_knots, guess, link_name);
%         solres = [solinfo{:}];
%         err = sum([solres.PoseErrorNorm])/length(solres)
%         
%     end
% end




% %%%%
cDat = lines(20);
picked_idx = 1;
angles = linspace(0,2*pi, 20);
for n = 1:1
    
    initial_surface_point = picked_pts.selections.points(picked_idx,:);
    mark_on_mesh(picked_pts.selections.faces(picked_idx), iiwa_patch);
    % [ distance, surf_target, surf_normal ] = point2trimesh_with_normals( initial_surface_point, faces, vertices, face_normals, vertex_normals );
    [result_path_pts, result_path_normals, rot_integrated] = integrate_velocity_over_surface(tspan, surface_vel, initial_surface_point, [0 0 1], 2*pi/3, geo_data.merged_iiwa(detail_level));
    
    % IK
    ik_idx = floor(linspace(1,length(tspan) - 1, 5*laps));
    iiwa_breaks = tspan(ik_idx);
    
    surface_to_origin_translation = trvec2tform(-result_path_pts(ik_idx,:));
    world_origin_to_goal_pt_translation = trvec2tform(contact_loc_over_time(ik_idx,:));
    surf_vel_untransformed_to_planar_vel = rotm2tform(permute(rot_integrated(:,:,ik_idx), [2,1,3])); % Gets transposed here.
    iiwa_knots = zeros(4,4,size(surf_vel_untransformed_to_planar_vel,3));
    for i = 1:size(surf_vel_untransformed_to_planar_vel,3)
        
        iiwa_knots(:,:,i) = dummy_link_tform_to_iiwa_model(rot_integrated(:,:,ik_idx(i)), result_path_pts(ik_idx(i),:), contact_loc_over_time(ik_idx(i),:));
%         world_origin_to_goal_pt_translation(:,:,i) * surf_vel_untransformed_to_planar_vel(:,:,i) * surface_to_origin_translation(:,:,i)*inv(tform_dummy_to_robot)*getTransform(iiwa.robot, home, link_name);
    end
    % NOTE THAT RigidBodyTree 0 pose is not the same as the graphical 0 pose.
    % getTransform(iiwa.robot, home, link_name) goes between them.
    
    % iiwa_knots(:,:,1) =
    [jnt_angles, solinfo] = make_multi_point_trajectory(iiwa, iiwa_knots, guess, link_name);
    
%     tmp_iiwa = IIWAImporter(scene_fig);
%     for k = 1:length(tmp_iiwa.link_patches)
%         tmp_iiwa.link_patches{k}.FaceAlpha = 1;
%         tmp_iiwa.link_patches{k}.FaceColor = cDat(n,:);
%     end
%     
%     for j = 1:length(tspan)
%         joint_pos_struct = interpolate_traj(tmp_iiwa, iiwa_breaks, jnt_angles, tspan(j));
%         display_at_pose(tmp_iiwa, joint_pos_struct);
%         drawnow;
%     end
%     for k = 1:length(tmp_iiwa.link_patches)
%         tmp_iiwa.link_patches{k}.FaceAlpha = 0.25;
%         
%     end
    %     for j = 1:6:length(jnt_angles)
    %         tmp_iiwa = IIWAImporter(scene_fig)
    %         for k = 1:length(tmp_iiwa.link_patches)
    %            tmp_iiwa.link_patches{k}.FaceAlpha = 0.25;
    %            tmp_iiwa.link_patches{k}.FaceColor = cDat(picked_idx,:);
    %         end
    %         display_at_pose(tmp_iiwa, jnt_angles{j});
    %     end
    solres = [solinfo{:}];
    err = sum([solres.PoseErrorNorm])/length(solres)
    any(~cellfun(@(stat)(strcmp(stat.Status, 'success')), solinfo)) % Were any stages not successful?
end


% Video recording if desired

% for i = 1:length(iiwa.link_patches)
%    iiwa.link_patches{i}.Visible = 'off';
% end

write_to_vid = false;
if write_to_vid
    framerate = 60;
    vid_writer = VideoWriter('rolly_link_vid2.avi'); % Convert for Slack with 'ffmpeg -i infile.avi youroutput.mp4'
    vid_writer.FrameRate = framerate;
    vid_writer.Quality = 100;
    open(vid_writer);
end
campos([1.6970    1.5293    0.9466]);
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
    
    
    joint_pos_struct = interpolate_traj(iiwa, iiwa_breaks, jnt_angles, tspan(i));
    display_at_pose(iiwa, joint_pos_struct);
    
    %     camvec = (angle2dcm(0,0,i/200, 'xyz')*(lin_vel(i,:)*1.2 + [0.0 0 .25])')';
    %     camtarget(pos(i,:));
    %     campos(pos(i,:) + camvec);
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
end