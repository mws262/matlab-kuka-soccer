% Runs both the QP and NLP forms of the figure 8 optimization.

close all; clear all;
addpath ../path_optim;

knots = [0,0;
    1, 1;
    0, 2;
    -1, 1;
    1, -1;
    0, -2;
    -1, -1;
    0, 0];

breaks = linspace(0, 10, size(knots,1))';

segs_between = 5;
[ppx, ppy] = qp_spline(breaks, knots, segs_between);

figure;
teval = linspace(breaks(1), breaks(end), 300);
ppxval = ppval(teval, ppx);
ppyval = ppval(teval, ppy);
plot(ppxval, ppyval);
daspect([1,1,1]);

[ppx, ppy] = nlp_spline(breaks, knots, segs_between, 1, ppx, ppy); % 3 segments in between knots. Highest weight on "forcing" linear sections.

figure;
teval = linspace(breaks(1), breaks(end), 300);
ppxval = ppval(teval, ppx);
ppyval = ppval(teval, ppy);
plot(ppxval, ppyval);
daspect([1,1,1]);