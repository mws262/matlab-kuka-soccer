function scene_fig = make_visualizer_scene()
scene_fig = figure(1);
scene_fig.NumberTitle = 'off';
scene_fig.Name = 'Matt''s not-a-Drake-Visualizer';
scene_fig.ToolBar = 'none';
scene_fig.MenuBar = 'none';
colormap(winter);
hold on;

% Floor plane representing surface that the ball is rolling on.
[floor_x, floor_y] = meshgrid(-10:0.5:10); % Generate x and y data
[floor_z] = zeros(size(floor_x, 1)); % Generate z data
floor_patch = patch(surf2patch(floor_x, floor_y, floor_z));
floor_patch.FaceColor = [0.8,0.8,0.6];
floor_patch.EdgeAlpha = 0.2;
floor_patch.FaceAlpha = 1;
floor_patch.SpecularStrength = 0.2;
floor_patch.DiffuseStrength = 0.9;

% Sky sphere. From my MatlabPlaneGraphics example set.
skymap = [linspace(1,0.8,10)',linspace(1,0.8,10)',linspace(1,0.95,10)'];

[skyX,skyY,skyZ] = sphere(100);
sky = surf(200*skyX,200*skyY,200*skyZ,'LineStyle','none','FaceColor','interp');
sky.AmbientStrength = 0.8;
colormap(skymap);

axis([-3, 3, -3, 3]);
daspect([1,1,1]);
ax = scene_fig.Children;

ax.Projection = 'perspective';
ax.Clipping = 'off';
ax.Visible = 'off';
scene_fig.Position = [0, 0, 1500, 975];
ax.CameraPosition = [-2.4, -1.6, 1.8];
ax.CameraTarget = [0, 0, 0];
scene_fig.WindowKeyPressFcn = @key_callback;
scene_fig.WindowScrollWheelFcn = @mousewheel_callback;
camva(40);
light1 = light();
light1.Position = [10,10,40];%ax.CameraPosition;
light1.Style = 'infinite';

return