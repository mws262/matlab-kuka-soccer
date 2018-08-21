function total_spline = spline_concat_in_dimension(xspline, yspline, varargin)
%SPLINE_CONCAT_IN_DIMENSION If we have a piecewise polynomial for x,y,z,
%make them into a single one which evaluates both dimensions. Breaks,
%order, and all sorts of other crap should match between the given x and y
%pps. If z spline is not given, it will be assumed 0.

if ~all(xspline.breaks == yspline.breaks)
   error('Breaks of given splines to concatenate do not match.');
end

if ~isempty(varargin)
    zspline = varargin{1};
else
    zspline.coefs = zeros(size(xspline.coefs));
end
coefs = zeros(size(xspline.coefs,1)*3, size(xspline.coefs, 2)); 
coefs(1:3:end,:) = xspline.coefs;
coefs(2:3:end,:) = yspline.coefs;
coefs(3:3:end,:) = zspline.coefs;
total_spline = ppmak(xspline.breaks, coefs,3);
end

