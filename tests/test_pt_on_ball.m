% Test mesh in contact with the ball as the ball rolls along a spline path.
% The contact location on the ball should obey the acceleration
% requirements. This is mostly to make sure that the geometry of the
% rolling contact is correct.

close all; clear all;

% Ball and scene
ball_radius = 0.1;
scene_fig = make_visualizer_scene();
[ball_patch, ball_verts_untransformed] = make_ball(ball_radius);

mesh_data = get_mesh_data('cube');
mesh_data.vertices = mesh_data.vertices .* [2, 0.2, 2]; % originally 0.1 HALF-extent.
cmap = flag(size(mesh_data.faces,1));

faces = mesh_data.faces;
vertices = mesh_data.vertices;
face_normals = mesh_data.face_normals;
vertex_normals = mesh_data.vertex_normals;

manipulator_patch = patch('Faces', faces, 'Vertices', vertices, 'FaceNormals', face_normals, 'VertexNormals', vertex_normals);
manipulator_patch.EdgeColor = [0.1, 0.1, 0.2];
manipulator_patch.FaceColor = 'flat';
manipulator_patch.FaceVertexCData = cmap;
manipulator_patch.FaceAlpha = 0.65;
manipulator_patch.BackFaceLighting = 'lit';
manipulator_tform = hgtransform;
manipulator_patch.Parent = manipulator_tform;

%% Pick a path polynomial.
path_pp = get_path_pp('large_circle', 5);

total_ts = 1000; % Total timesteps to evaluate at.
approach_ang = 0;
arc_angle = pi/3; % Angle along the possible arc of the ball to contact.
initial_surface_point = [0,1,0]; % Initial point on the surface to project down.

%% Evaluate ball and contact point quantities.
[tspan, posspan, velspan, accelspan, omegaspan, quatspan, ...
    world_contact_loc_desired_fcn, contact_loc_desired_rel_com_fcn, contact_loc_world_vel] = evaluate_spline(path_pp, ball_radius, total_ts);

world_contact_desired_span = world_contact_loc_desired_fcn(arc_angle*ones(size(tspan)));
contact_desired_rel_com_span = contact_loc_desired_rel_com_fcn(arc_angle*ones(size(tspan)));
contact_loc_vel = contact_loc_world_vel(arc_angle*ones(size(tspan)), zeros(size(tspan)));

% Just how the contact point moves around on the surface of the ball
% relative to its center. Nothing to do with ball rotation.
contact_loc_vel_rel_center = contact_loc_vel - velspan; % For a plane, all rotation comes from this component (I think).
% w = rxv/|r|^2 world angular rate of pusher.
pusher_angular_rate = cross(contact_desired_rel_com_span, contact_loc_vel_rel_center)./ball_radius^2;

vec1 = dot(contact_desired_rel_com_span(1,:)/ball_radius,[1,0,0]) * [1,0,0] + dot(contact_desired_rel_com_span(1,:)/ball_radius,[0,1,0]) * [0,1,0];
rot1 = get_rotation_from_vecs(vec1, contact_desired_rel_com_span(1,:)/ball_radius) * get_rotation_from_vecs([0;1;0], vec1);
quatinit = rotm2quat(rot1);
quats = quatintegrate(tspan, quatinit, pusher_angular_rate);
% rotated_up = quatrotate(quats, [0,0,1])


up_vector_span = contact_desired_rel_com_span/ball_radius;
surface_vel_span = cross(omegaspan, contact_desired_rel_com_span, 2);

contact_pt_pl = plot(0,0,'.g','MarkerSize', 40);
integrated_contact_pt_pl = plot(0,0,'.y', 'MarkerSize', 40);
init_pt = world_contact_desired_span(1,:);

% Come up with how much of the contact point velocity and ball surface velocity
% align. If they align perfectly, then the box can just follow the surface
% without changing the contact point on the box. If they don't align (or
% anti-align), then we need components from both.

ptvelnorms = sqrt(sum((contact_loc_vel - velspan).^2,2));
surfvelnorms = sqrt(sum((surface_vel_span).^2,2));
aligning_vel = dot(contact_loc_vel - velspan, surface_vel_span./surfvelnorms, 2);
net_box_vel = contact_loc_vel + surface_vel_span - aligning_vel.*surface_vel_span./surfvelnorms;
integrated_pt_span = cumtrapz(tspan, net_box_vel) + init_pt;

% center_of_box = tform2trvec(trvec2tform(integrated_pt_span(1,:))*rotm2tform(quat2rotm(quats(1,:))'))
draw_path_and_accel(posspan, accelspan, 3);
for i = 1:10:length(tspan) - 1
    quat = quatspan(i,:);
    ball_patch.Vertices = quatrotate(quat, ball_verts_untransformed) + repmat(posspan(i,:)  + [0, 0, ball_radius], [size(ball_verts_untransformed,1),1]);
        
    % Show contact point.
    contact_pt_pl.XData = world_contact_desired_span(i,1);
    contact_pt_pl.YData = world_contact_desired_span(i,2);
    contact_pt_pl.ZData = world_contact_desired_span(i,3);
    
    integrated_contact_pt_pl.XData = integrated_pt_span(i,1);
    integrated_contact_pt_pl.YData = integrated_pt_span(i,2);
    integrated_contact_pt_pl.ZData = integrated_pt_span(i,3);
    
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
    
    manipulator_tform.Matrix = trvec2tform(integrated_pt_span(i,:))*rotm2tform(quat2rotm(quats(i,:))')*trvec2tform([0, 0.02, 0]);
    drawnow;
   % pause(0.05);
end



