function [accel_zero_break_start, accel_zero_break_end] = find_zero_accel_breaks(poly_accel_x, poly_accel_y, total_breaks, zero_accel_tol)
%FIND_ZERO_ACCEL_BREAKS Figure out which breaks in a spline correspond to
%low/zero acceleration polynomials.
%   poly_accel_x - Piecewise polynomial for x acceleration.
%   poly_accel_y - Piecewise polynomial for y acceleration.
%   total_breaks - All polynomial timing switches, not just ones for low
%   accel.
%   zero_accel_tol - Limit in higher order polynomial terms below which
%   they are considered negligible.

    max_accel_coef_x = max(abs(poly_accel_x.coefs),[],2); % For identifying regions where the acceleration goes to 0.
    max_accel_coef_y = max(abs(poly_accel_y.coefs),[],2); % For identifying regions where the acceleration goes to 0.
    accel_zero_segments = abs(max_accel_coef_x) + abs(max_accel_coef_y) < zero_accel_tol; % 1 if it is a zero-accel region.
    
    start_zero_segs = false(length(accel_zero_segments));
    start_zero_segs(1) = accel_zero_segments(1);
    for i = 2:length(accel_zero_segments)
        if accel_zero_segments(i) && ~accel_zero_segments(i-1)
            start_zero_segs(i) = true;
        end
    end
    
    end_zero_segs = false(length(accel_zero_segments));
    end_zero_segs(end) = accel_zero_segments(end);
    for i = 1:length(accel_zero_segments) - 1
        if accel_zero_segments(i) && ~accel_zero_segments(i+1)
            end_zero_segs(i) = true;
        end
    end
    
    start_breaks = total_breaks(1:end-1);
    end_breaks = total_breaks(2:end);
    
    accel_zero_break_start = start_breaks(start_zero_segs);
    accel_zero_break_end = end_breaks(end_zero_segs);
end
