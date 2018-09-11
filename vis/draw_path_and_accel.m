function [path_line, accel_arrows] = draw_path_and_accel(positions, accelerations, arrow_reduction)
% DRAW_PATH_AND_ACCEL Draws the a path in 3D space and matching
% accelerations along this path as arrows.
%
%   [path_line, accel_arrows] = DRAW_PATH_AND_ACCEL(posspan, accelspan, arrow_reduction)
%
%   Inputs:
%       `positions` -- n x 3 matrix representing positions to draw in 3D.
%       Will draw as a thick line, presumeably on the ground.
%       `accelerations` -- n x 3 matrix representing accelerations along
%       the path defined by positions. These will be drawn as arrows along
%       the path. Must match the dimension of positions.
%       `arrow_reduction` -- Thins out the number of arrows displayed if it
%       is too dense. Should be a whole number >=1. 1 is no reduction,
%       higher is more reduction.
%
%   Outputs:
%       `path_line` -- Graphics object for the path line drawn.
%       `accel_arrows` -- Graphics object (quiver) drawn for the
%       accelerations.
%
%   See also MAKE_VISUALIZER_SCENE, PLOT3, QUIVER3.
%

hold on;
path_line = plot3(positions(:,1), positions(:,2), positions(:,3), 'LineWidth', 2);
accel_arrows = quiver3(positions(1:arrow_reduction:end,1), positions(1:arrow_reduction:end,2), positions(1:arrow_reduction:end,3), ...
    accelerations(1:arrow_reduction:end,1), accelerations(1:arrow_reduction:end,2), accelerations(1:arrow_reduction:end,3));