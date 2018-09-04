
close all; clear all;

fig = figure;
ax = axes;
ax.Visible = 'off';
fig.Color = [1 1 1];
fig.Position = [0 0 600 600];

eom_str = {'$[m + \frac{I}{R^2}]\ddot{x} = F_{a^*}$'};
eom_txt = text(0.15, 0.85, eom_str);
eom_txt.Interpreter = 'latex';
eom_txt.FontSize = 32;

prop_str = {'$\ddot{x}\propto{}F_{a^*}$'};
prop_txt = text(0.15, 0.5, prop_str);
prop_txt.Interpreter = 'latex';
prop_txt.FontSize = 32;

cost_fun_str = {'$\sum_{2}^{#segments -1}  \int_{t_{n-1}}^{t_n} |accel_{n-1}|^2 dt\int_{t_{n-1}}^{t_n} |accel_{n-1}|^2 dt\int_{t_{n-1}}^{t_n} |accel_{n-1}|^2 dt$'};
cost_fun_txt = text(0.15, 0.2, cost_fun_str);
cost_fun_txt.Interpreter = 'latex';
cost_fun_txt.FontSize = 20;

% save2pdf('../../data/images/pos_accel_poly.pdf', fig, 600);