close all; clear all;

fig = figure;
ax = axes;
ax.Visible = 'off';
fig.Color = [1 1 1];
fig.Position = [0 0 600 600];

highlight_poly_txt = {'Positions:', '$x_n(t) = a_{x_n}t^3 + b_{x_n}t^2 + c_{x_n}t + d_{x_n}$',  '$y_n(t) = a_{y_n}t^3 + b_{y_n}t^2 + c_{y_n}t + d_{y_n}$'};
highlight_poly = text(0.15, 0.85, highlight_poly_txt);
highlight_poly.Interpreter = 'latex';
highlight_poly.FontSize = 20;

highlight_poly_txt2 = {'Accelerations:', '$\ddot{x}_n(t) = 6a_{x_n}t + 2b_{x_n}$',  '$\ddot{y}_n(t) = 6a_{y_n}t + 2b_{y_n}$'};
highlight_poly2 = text(0.15, 0.5, highlight_poly_txt2);
highlight_poly2.Interpreter = 'latex';
highlight_poly2.FontSize = 20;

save2pdf('../../data/images/pos_accel_poly.pdf', fig, 600);