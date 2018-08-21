% Runs both the QP and NLP forms of the figure 8 optimization.

close all; clear all;
addpath ../path_optim;

% fig 8
knots = [0,0;
    1, 1;
    0, 2;
    -1, 1;
    1, -1;
    0, -2;
    -1, -1;
    0, 0];

% triangle
offset = [0.48, 0];
scale = 0.15;
knots = fliplr([ 0, sqrt(3); 1, 0; -1, 0; 0, sqrt(3)]);

breaks = linspace(0, 10, size(knots,1))';

segs_between = 3;
[ppx, ppy] = qp_spline(breaks, knots, segs_between);

figure;
teval = linspace(breaks(1), breaks(end), 300);
ppxval = ppval(teval, ppx);
ppyval = ppval(teval, ppy);
plot(ppxval, ppyval);
daspect([1,1,1]);

[ppx, ppy] = nlp_spline(breaks, knots, segs_between, 1, ppx, ppy); % 3 segments in between knots. Highest weight on "forcing" linear sections.
ppx.coefs = ppx.coefs * scale; % For some reason, adjusting scale and offset before the optimization does weird things. Maybe because of convergence tolerances?
ppy.coefs = ppy.coefs * scale;
ppx.coefs(:,end) = ppx.coefs(:,end) + offset(1);
ppy.coefs(:,end) = ppy.coefs(:,end) + offset(2);

triangle_spline = spline_concat_in_dimension(ppx,ppy);
save('../data/triangle_position_spline.mat', 'triangle_spline');
figure;
teval = linspace(breaks(1), breaks(end), 300);
ppxval = ppval(teval, ppx);
ppyval = ppval(teval, ppy);
plot(ppxval, ppyval);
daspect([1,1,1]);

% Split the fully piecewise polynomial into sections. Makes a separate
% piecewise polynomial, starting at time 0, for each contact region.
[zero_starts, zero_ends, contact_polys, shifted_pp] = find_zero_accel_breaks_from_pos_pp(triangle_spline, 3, 1e-6);

figure;
hold on;
daspect([1,1,1]);
for i = 1:length(contact_polys)
   poly_val = ppval(contact_polys(i), linspace(0, contact_polys(i).breaks(end), 100))';
   plot(poly_val(:,1), poly_val(:,2));
end

starts_eval = ppval(triangle_spline, zero_starts)';
ends_eval = ppval(triangle_spline, zero_ends)';
plot(starts_eval(:,1), starts_eval(:,2), '.g', 'MarkerSize', 20);
plot(ends_eval(:,1), ends_eval(:,2), '.r', 'MarkerSize', 20);

% If we shift the start of the whole piecewise poly to align with the start
% of a contact section, make sure it still works/makes sense.
figure;
eval_shifted = ppval(shifted_pp, linspace(0, shifted_pp.breaks(end), 500))';
plot(eval_shifted(:,1), eval_shifted(:,2));
hold on;
plot(eval_shifted(1,1), eval_shifted(1,2), '.g', 'MarkerSize', 40);
plot(eval_shifted(end,1), eval_shifted(end,2), '.r', 'MarkerSize', 20);



