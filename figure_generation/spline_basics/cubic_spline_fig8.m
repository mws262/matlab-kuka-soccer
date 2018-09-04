close all; clear all;
addpath ../../;
addpath ../../vis/;
addpath ../../geometry/;
addpath ../../derived_autogen/;
addpath ../../data/;
addpath ../../dynamics/;
addpath ../../util/;

save_dir = '../../data/images/';

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
    0, 0, height];

num_knots = size(knots,1);
breaks = linspace(0, time_scaling, num_knots);

simple_spl = csape(breaks, knots', 'periodic');

teval = linspace(min(breaks), max(breaks), 250);
spl_eval = ppval(simple_spl, teval)';


fig = figure;
fig.Position = [0,0,800,800];
cmap = jet(num_knots);
hold on;
ax = fig.Children(end);
ax.Visible = 'off';

ax.XLim = [min(knots(:,1)), max(knots(:,1))] + [-1, 1] * 0.1;
ax.YLim = [min(knots(:,2)), max(knots(:,2))] + [-1, 1] * 0.1;

daspect([1,1,1]);

fig.Color = [1,1,1];
%% Show drawing of knot points.
for i = 1:num_knots
    plot(knots(i,1), knots(i,2), '.', 'Color', cmap(i,:), 'MarkerSize', 50);
    lab = text(knots(i,1), knots(i,2), [num2str(i), '.']);
    lab.FontSize = 24;
    lab.HorizontalAlignment = 'center';
    if i == num_knots
       lab.Position = lab.Position + [-0.08, 0, 0];
    elseif i == 1
        lab.Position = lab.Position + [0.08, 0, 0];
    else 
        lab.Position = lab.Position + [0, 0.08, 0];
    end
    
    drawnow;
      % Capture the plot as an image 
      frame = getframe(fig); 
      im = frame2im(frame); 
      [imind,cm] = rgb2ind(im,256); 
      % Write to the GIF File 
      if i == 1 
          imwrite(imind, cm, [save_dir, 'spline_basics_fig_8_knots_anim.gif'] ,'gif', 'Loopcount' ,inf); 
      else 
          imwrite(imind, cm, [save_dir, 'spline_basics_fig_8_knots_anim.gif'] ,'gif', 'WriteMode', 'append'); 
      end 
end

save2pdf([save_dir, 'spline_basics_fig_8_knots.pdf'], fig, 600);


%% Show drawing of spline.
spl_pl = plot(spl_eval(:,1),spl_eval(:,2), 'LineWidth', 3);
for i = 1:4 % add multiple frames so it pauses on this a little bit.
    drawnow;
    % Capture the plot as an image
    frame = getframe(fig);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    imwrite(imind, cm, [save_dir, 'spline_basics_fig_8_knots_anim.gif'] ,'gif', 'WriteMode', 'append');
end

save2pdf([save_dir, 'spline_basics_fig_8_knots_w_spline.pdf'], fig, 600);

%% Plot fig8 with inward accels
inward_accel_fig = figure;
inward_accel_fig.Position = [0,0,800,800];
cmap = jet(num_knots);
hold on;
ax_accel = inward_accel_fig.Children(end);
ax_accel.Visible = 'off';

ax_accel.XLim = [min(knots(:,1)), max(knots(:,1))] + [-1, 1] * 0.1;
ax_accel.YLim = [min(knots(:,2)), max(knots(:,2))] + [-1, 1] * 0.1;
spl_pl_for_accel = plot(spl_eval(:,1),spl_eval(:,2), 'LineWidth', 3);
spl_pl_for_accel.Color = spl_pl.Color;
daspect([1,1,1]);

inward_accel_fig.Color = [1,1,1];

spl_eval_accel = ppval(fnder(simple_spl,2), teval)';
skip_int = 3;
accel_arrows = quiver(spl_eval(1:skip_int:end,1), spl_eval(1:skip_int:end,2), spl_eval_accel(1:skip_int:end,1), spl_eval_accel(1:skip_int:end,2));
save2pdf([save_dir, 'spline_basics_fig_8_spline_with_accel.pdf'], inward_accel_fig, 600);





