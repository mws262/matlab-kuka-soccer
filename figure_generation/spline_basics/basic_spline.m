%% Simple spline example. Just shows a segment with polynomials labeled.

close all; clear all;
addpath ../../;
addpath ../../vis/;
addpath ../../geometry/;
addpath ../../derived_autogen/;
addpath ../../data/;
addpath ../../dynamics/;
addpath ../../util/;

save_dir = '../../data/images/';

fig = figure;
fig.Position = [0, 0, 1000, 800];
fig.Color = [1,1,1];
ax = axes;
ax.Visible = 'off';

% Make up some knot points.
knots = [-1, 0; 1, 0.1; 2, 1; 4, 1.2];
tend = 1;

daspect([1 1 1]);
ax.XLim = [min(knots(:,1)), max(knots(:,1))] + [1, -1];
ax.YLim = [min(knots(:,2)), max(knots(:,2))] + [-0.5, 0.5];

breaks = linspace(0, tend, size(knots, 1));

spl = spline(breaks, knots');
teval = linspace(0,tend, 200);

xeval = ppval(spl, teval)';

% Spline plotting.
hold on;
mid_idx = teval> breaks(2) & teval < breaks(3); % Section between center knots.
center_section = plot(xeval(mid_idx, 1), xeval(mid_idx, 2), 'LineWidth', 4);
low_sections = plot(xeval( teval < breaks(2), 1), xeval(teval < breaks(2), 2), 'LineStyle', '--', 'LineWidth', 3, 'Color', [0.6, 0.6, 1]);
high_sections = plot(xeval( teval > breaks(3), 1), xeval(teval > breaks(3), 2), 'LineStyle', '--', 'LineWidth', 3, 'Color', [0.6, 0.6, 1]);
knot_plot = plot(knots(:,1), knots(:,2), '.b', 'MarkerSize', 30);

% Draw latex polynomials
highlight_poly_txt = {'$x_n(t) = a_{x_n}t^3 + b_{x_n}t^2 + c_{x_n}t + d_{x_n}$',  '$y_n(t) = a_{y_n}t^3 + b_{y_n}t^2 + c_{y_n}t + d_{y_n}$'};
highlight_poly = text(0.15, 0.85, highlight_poly_txt);
highlight_poly.Interpreter = 'latex';
highlight_poly.FontSize = 20;

lower_poly_txt = {'$x_{n-1}(t) = a_{x_{n-1}}t^3 + ...$', '$y_{n-1}(t) = a_{y_{n-1}}t^3 + ...$'};
lower_poly = text(-0., -0.35, lower_poly_txt);
lower_poly.Interpreter = 'latex';
lower_poly.FontSize = 16;
lower_poly.Color = [0.5, 0.5, 0.5];

upper_poly_txt = {'$x_{n+1}(t) = a_{x_{n+1}}t^3 + ...$', '$y_{n+1}(t) = a_{y_{n+1}}t^3 + ...$'};
upper_poly = text(2.25, 1, upper_poly_txt);
upper_poly.Interpreter = 'latex';
upper_poly.FontSize = 16;
upper_poly.Color = [0.5, 0.5, 0.5];

% Annotations
[arr1x, arr1y] = ds2nfu(ax, [1.3, knots(2,1) + 0.08], [-0.2, knots(2,2) ]);
note1 = annotation('textarrow', arr1x, arr1y);
[arr1x, arr1y] = ds2nfu(ax, [1.3, knots(2,1) - 0.08], [-0.2, knots(2,2) - 0.1]);
annotation('textarrow', arr1x, arr1y);
note1.String = {'Match velocities', 'and accelerations.'};
note1.FontSize = 16;
note1.FontName = 'times';

% Having a second one pointing to another knot point 
[arr2x, arr2y] = ds2nfu(ax, [knots(3,1), knots(3,1)], [0.4 + knots(3,2), knots(3,2) + 0.05]);
note2 = annotation('textarrow', arr2x, arr2y);
note2.String = {'Break:     $t_{n+1}$', 'Knot: $[x_n(t_{n+1}), y_n(t_{n+1})]$'}
note2.Interpreter = 'latex';
note2.HorizontalAlignment = 'left';
note2.FontSize = 16;
note2.FontName = 'times';

% Axis
xaxis = annotation('textarrow', [0.8, 0.9], [0.1, 0.1]);
yaxis = annotation('textarrow', [0.8, 0.8], [0.1, 0.2]);
xaxis.String = '$x$';
xaxis.Interpreter = 'latex';
xaxis.FontSize = 18;


yaxis.String = '$y$';
yaxis.Interpreter = 'latex';
yaxis.FontSize = 18;

drawnow;

save2pdf([save_dir, 'spline_basics.pdf'], fig, 600);

