% Run optimization of the figure 8 in its nlp form. This means that it has
% the force-adjacent-section-product cost meant to cause some sections to
% be linear.

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

[ppx, ppy] = nlp_spline(breaks, knots, 3, 1); % 3 segments in between knots. Highest weight on "forcing" linear sections.

teval = linspace(breaks(1), breaks(end), 300);
ppxval = ppval(teval, ppx);
ppyval = ppval(teval, ppy);
plot(ppxval, ppyval);
daspect([1,1,1]);