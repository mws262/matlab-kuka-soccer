function [path_line, accel_arrows] = draw_path_and_accel(posspan, accelspan, arrow_reduction)
% [path_line, accel_arrows] = DRAW_PATH_AND_ACCEL(posspan, accelspan, arrow_reduction)
%   Draws the path as a thick line on the ground and accelerations as
%   quiver arrows. arrow_reduction will thin out the number of arrows
%   displayed. Higher is more reduction. Should be a whole number >= 1.

hold on;
path_line = plot3(posspan(:,1), posspan(:,2), posspan(:,3), 'LineWidth', 2);
accel_arrows = quiver3(posspan(1:arrow_reduction:end,1), posspan(1:arrow_reduction:end,2), posspan(1:arrow_reduction:end,3), ...
    accelspan(1:arrow_reduction:end,1), accelspan(1:arrow_reduction:end,2), accelspan(1:arrow_reduction:end,3));