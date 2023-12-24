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

teval = linspace(min(breaks), max(breaks), 250)';
spl_eval = ppval(simple_spl, teval)';

fig = figure;
fig.Color = [1,1,1];
fig.Position = [100,100, 1200, 600];
cmap = lines(10);


splsmooth = qp_collocation(breaks, knots, 2, 0.0);
splsmoothddt = fnder(splsmooth, 2);
splsmoothEval = ppval(splsmooth, teval)';
splsmoothddtEval = ppval(splsmoothddt, teval)';
ax1 = subplot(1,2,1);
hold on;
plot(splsmoothEval(:,1), splsmoothEval(:,2), 'LineWidth', 3, 'Color', cmap(2,:));
quiver(splsmoothEval(1:2:end,1), splsmoothEval(1:2:end,2), splsmoothddtEval(1:2:end,1)/20, splsmoothddtEval(1:2:end,2)/20, 'Autoscale', 'on');
plot(knots(:,1), knots(:,2), '.', 'MarkerSize', 30, 'Color', cmap(1,:));
axis equal;
ax1.Visible = false;


splsmooth = qp_collocation(breaks, knots, 4, 1);
splsmoothddt = fnder(splsmooth, 2);
splsmoothEval = ppval(splsmooth, teval)';
splsmoothddtEval = ppval(splsmoothddt, teval)';
ax2 = subplot(1,2,2);
hold on;
plot(splsmoothEval(:,1), splsmoothEval(:,2), 'LineWidth', 3, 'Color', cmap(2,:));
quiver(splsmoothEval(1:2:end,1), splsmoothEval(1:2:end,2), splsmoothddtEval(1:2:end,1)/20, splsmoothddtEval(1:2:end,2)/20, 'Autoscale', 'on');
plot(knots(:,1), knots(:,2), '.', 'MarkerSize', 30, 'Color', cmap(1,:));
axis equal;
ax2.Visible = false;


