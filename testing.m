[X,Y,Z] = peaks(3);

center = [X(2,2), Y(2,2), Z(2,2)];

Xdiff = X - center(1);
Ydiff = Y - center(2);
Zdiff = Z - center(3);

% Zdiff./sqrt(Xdiff.^2 + Ydiff.^2)

diffsfromcenter = [Xdiff(:), Ydiff(:), Zdiff(:)]