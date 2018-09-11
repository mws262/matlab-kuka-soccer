function total_spline = spline_concat_in_dimension(xspline, yspline, zspline)
% SPLINE_CONCAT_IN_DIMENSION If we have a piecewise polynomial for x,y,z,
% make them into a single one which evaluates both dimensions. Breaks,
% order, and all sorts of other crap should match between the given x and y
% pps. If z spline is not given, it will be assumed 0.
%
%   total_spline = SPLINE_CONCAT_IN_DIMENSION(xspline, yspline, zspline)
%   total_spline = SPLINE_CONCAT_IN_DIMENSION(xspline, yspline)
%
%   Inputs:
%       `xspline` - 1D spline representing the x-coordinate.
%       `yspline` - 1D spline representing the x-coordinate.
%       `zspline` - (OPTIONAL) 1D spline representing the x-coordinate. 0
%       is used if no zspline is provided.
%   Outputs:
%       `total_spline` -- 3D spline containing x, y, and z components.
%
%   See also SPLINE, CSAPE, PPMAK, NLP_SPLINE, QP_SPLINE.
%

time_diff_tol = 1e-6; % Given splines must have VERY similar time breaks already.

validateattributes(xspline, {'struct'}, {});
validateattributes(yspline, {'struct'}, {});
assert(xspline.dim == 1, 'Input splines should be single-dimensional.');
assert(yspline.dim == 1, 'Input splines should be single-dimensional.');
assert(xspline.order == yspline.order, 'Input splines must have the same order.');
assert_near(xspline.breaks, yspline.breaks, time_diff_tol, 'Given splines do not have close enough time breaks to concatenate.');

if nargin == 3 % Optional zspline 3rd argument.
    validateattributes(zspline, {'struct'}, {});
    assert(zspline.dim == 1, 'Input splines should be single-dimensional.');
else
    zspline.coefs = zeros(size(xspline.coefs));
end

coefs = zeros(size(xspline.coefs,1)*3, size(xspline.coefs, 2)); 
coefs(1:3:end,:) = xspline.coefs;
coefs(2:3:end,:) = yspline.coefs;
coefs(3:3:end,:) = zspline.coefs;
total_spline = ppmak(xspline.breaks, coefs,3);
end

