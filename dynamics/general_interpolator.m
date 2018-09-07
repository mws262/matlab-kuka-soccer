classdef general_interpolator < handle
    %% General purpose interpolator.
    % Data can be added again and again as long as the times are
    % monotonically increasing. If the data is one-dimensional, then the
    % first dimension should be time. If it is higher dimensional (e.g.
    % rotation matrices), then the last dimension should be time.
    
    properties
        dspan_complete;
        tspan_complete;
        data_dimensions;
    end
    methods
        function obj = general_interpolator(data_dimensions)
            validateattributes(data_dimensions, {'numeric'}, {'integer', 'positive'});
            obj.data_dimensions = data_dimensions;
        end
        
        function add_to_end_at_time(obj, tspan, dspan) % Add to the end while preserving the existing times in tspan rather than offsetting.

            validateattributes(tspan, {'single', 'double'}, {'vector', 'increasing', 'real', 'nonnan'});
            validateattributes(dspan, {'single', 'double'}, {'real', 'nonnan'});
            
            if isrow(tspan)
                tspan = tspan';
            end
            
            if ~isempty(obj.tspan_complete) & tspan(1) <= obj.tspan_complete(end)
                error('Tried to add a timespan at the end which is not greater than the existing times.');
            end
            
            dims = obj.ndims_with_one(obj.data_dimensions);
            indatsz = size(dspan);
            if (dims > 1 && any(indatsz(1:end - 1) ~= obj.data_dimensions)) || (dims == 1 && size(dspan,2) ~= obj.data_dimensions)
                error('Tried to add new data to a general_interpolator which did not match the provided dimensions.');
            end
            
            if dims > 1 && length(tspan) > 1
                dspan = permute(dspan, [ndims(dspan), 1:ndims(dspan)-1]);
                indatsz = size(dspan);
            end
            
            if length(tspan) == 1 % Need to add that singleton dimension if it's only a single time element.
                dspan_resize = reshape(dspan, [1, prod(indatsz(1:end))]); % Resizing to two dimensions for easy interpolation. 1st dim is always time.
            else
                dspan_resize = reshape(dspan, [indatsz(1), prod(indatsz(2:end))]); % Resizing to two dimensions for easy interpolation. 1st dim is always time.
            end
            
            obj.tspan_complete = [obj.tspan_complete; tspan];
            obj.dspan_complete = [obj.dspan_complete; dspan_resize];
        end
        function dat_at_time = get_at_time(obj, tget)
            
            validateattributes(tget, {'single', 'double'}, {'nonnan', 'scalar', 'real', '<=', obj.tspan_complete(end), '>=', obj.tspan_complete(1)});
            
            dat_interp = interp1(obj.tspan_complete, obj.dspan_complete, tget);
            
            if obj.ndims_with_one(obj.data_dimensions) > 1
                dat_interp = permute(dat_interp, [2:obj.data_dimensions, 1]); % Put time dimension at the end again.
                dat_at_time = squeeze(reshape(dat_interp, [1,obj.data_dimensions]));
            else
                dat_at_time = dat_interp;
            end
            
            validateattributes(dat_at_time, {'single', 'double'}, {'real'});
            
        end
        % Same as build-in ndims, but can return 1 if there is only 1
        % non-singleton dimension. Will not return less than 1.
        function dims = ndims_with_one(obj, arr)
            dims = max(length(arr(arr ~= 1)),1);
        end
    end
end