function [accel_zero_break_start, accel_zero_break_end, contact_polys, shifted_position_pp] = find_zero_accel_breaks_from_pos_pp(pp_pos, dim, zero_accel_tol)
%FIND_ZERO_ACCEL_BREAKS Figure out which breaks in a spline correspond to
%low/zero acceleration polynomials. Also makes a shifted version of the
%piecewise polynomial that starts with the beginning of a contact region.
%   pp_pos - Position piecewise polynomial for the whole ball path.
%   dim - Number of dimensions, e.g. 3 for x y z.
%   zero_accel_tol - Limit in higher order polynomial terms below which
%   they are considered negligible.
%   Returns:
%   accel_zero_break_start - Beginning times of the ballistic sections.
%   accel_zero_break_end - End times of the ballistic sections.
%   contact_polys - Struct array of piecewise polynomials for all of the
%   contacting regions. These are shifted to have breaks beginning at time
%   0.

accel_pp = fnder(pp_pos,2);
total_breaks = pp_pos.breaks;

max_accel_coefs = max(abs(accel_pp.coefs),[],2); % For identifying regions where the acceleration goes to 0.
accel_zero_segments = sum(reshape(max_accel_coefs, [dim, length(max_accel_coefs)/dim])',2) < zero_accel_tol;

start_zero_segs = false(length(accel_zero_segments),1);
start_zero_segs(1) = accel_zero_segments(1);
for i = 2:length(accel_zero_segments)
    if accel_zero_segments(i) && ~accel_zero_segments(i-1)
        start_zero_segs(i) = true;
    end
end

end_zero_segs = false(length(accel_zero_segments),1);
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


zero_start_idx = find(start_zero_segs);
zero_end_idx = find(end_zero_segs);

%% Shift the whole thing so it starts with a contact region.
if isempty(zero_end_idx)
    shifted_position_pp = pp_pos; % If the entire thing is one large contact, nothing to do here.
    contact_polys = pp_pos;
else
    shifted_coeffs = pp_pos.coefs(zero_end_idx(1)*3 + 1:end,:);
    shifted_coeffs = [shifted_coeffs; pp_pos.coefs(1:zero_end_idx(1)*3,:)];
    %     shifted_coeffs = round(shifted_coeffs, 7);
    
    %     shifted_coeffs(1:3,2) = sign(shifted_coeffs(1:3,1)).*sign(shifted_coeffs(1:3,2)) .* shifted_coeffs(1:3,2);
    
    shifted_breaks = pp_pos.breaks(zero_end_idx(1) + 1:end);
    shifted_breaks = shifted_breaks - pp_pos.breaks(zero_end_idx(1) + 1);
    % I feel like this is dumb. Revisit if time.
    shifted_breaks = [shifted_breaks, cumsum(diff(pp_pos.breaks(1:zero_end_idx(1) + 1))) + shifted_breaks(end)]; % assumes first break is always 0.
    shifted_position_pp = ppmak(shifted_breaks, shifted_coeffs,3);
    
    %% Make piecewise polynomials separately for each contact location, using the shifted pp
    contact_polys = {};
    shifted_accel_pp = fnder(shifted_position_pp, 2);
    shifted_breaks = shifted_position_pp.breaks;
    
    shifted_max_accel_coefs = max(abs(shifted_accel_pp.coefs),[],2); % For identifying regions where the acceleration goes to 0.
    
    accel_not_zero_shifted = sum(reshape(shifted_max_accel_coefs, [dim, length(shifted_max_accel_coefs)/dim])',2) >= zero_accel_tol;
    
    % Identify the beginning and end of contact on the shifted pp.
    start_contact_segs = false(length(accel_not_zero_shifted),1);
    start_contact_segs(1) = accel_not_zero_shifted(1);
    for i = 2:length(accel_not_zero_shifted)
        if accel_not_zero_shifted(i) && ~accel_not_zero_shifted(i-1)
            start_contact_segs(i) = true;
        end
    end
    
    end_contact_segs = false(length(accel_not_zero_shifted),1);
    end_contact_segs(end) = accel_not_zero_shifted(end);
    for i = 1:length(accel_not_zero_shifted) - 1
        if accel_not_zero_shifted(i) && ~accel_not_zero_shifted(i+1)
            end_contact_segs(i) = true;
        end
    end
    
    contact_start_idx = find(start_contact_segs);
    contact_end_idx = find(end_contact_segs);
    
    % Make the pps for each contact section. Preserve their normal break
    % timings.
    for i = 1:length(contact_start_idx)
        st_idx = contact_start_idx(i);
        end_idx = contact_end_idx(i);
        section_coeffs = shifted_position_pp.coefs((st_idx - 1) * 3 + 1 : (end_idx - 1) * 3 + 3, :);
        section_breaks = shifted_breaks(st_idx:end_idx + 1);
        contact_polys{end + 1} = ppmak(section_breaks, section_coeffs, dim);
    end
    
    contact_polys = [contact_polys{:}];
end

end
