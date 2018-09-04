classdef general_interpolator < handle
    properties
        dspan_complete;
        tspan_complete;
        data_dimensions;
    end
    methods
        function obj = general_interpolator(data_dimensions)
            obj.data_dimensions = data_dimensions;
        end
        % This is a terrible method because adding to the end without
        % preserving the new data's first time is a terrible idea.
%         function add_to_end(obj, tspan, dspan)
%             if isrow(tspan)
%                 tspan = tspan';
%             end
%             
%             if size(dspan,1) ~= length(tspan)
%                error('Given dspan should have rows matching time samples. May need to transpose.'); 
%             end
%             
%             indatsz = size(dspan);
%             if any(indatsz(2:end) ~= obj.data_dimensions)
%                 error('Tried to add new data to a general_interpolator which did not match the provided dimensions.');
%             end
%             dspan_resize = reshape(dspan, [indatsz(1), prod(indatsz(2:end))]); % Resizing to two dimensions for easy interpolation. 1st dim is always time.
%             % Average the difference in time between the last two old times
%             % and the first two new times to decide how far after the old
%             % data to put the new. If times are close to evenly
%             % distributed, it will probably be good enough.
%             if isempty(obj.tspan_complete)
%                 toffset = -tspan(1);
%             else
%                 toffset = (obj.tspan_complete(end) - obj.tspan_complete(end - 1) + tspan(2) - tspan(1))/2 + obj.tspan_complete(end) - tspan(1);
%             end
%             obj.tspan_complete = [obj.tspan_complete; tspan + toffset];
%             obj.dspan_complete = [obj.dspan_complete; dspan_resize];
%         end

        % Add to the dataset with timings in tspan preserved. If data is 2d
        % first dimension is expected to correspond with time. If the data
        % is higher-dimension, then the LAST dimension should be time (to
        % comply with other MATLAB convention).
        function add_to_end_at_time(obj, tspan, dspan) % Add to the end while preserving the existing times in tspan rather than offsetting.
            if isrow(tspan)
                tspan = tspan';
            end
            if ~isempty(obj.tspan_complete) & tspan(1) <= obj.tspan_complete(end)
               error('Tried to add a timespan at the end which is not greater than the existing times.'); 
            end
            
            indatsz = size(dspan);
            if (ndims(dspan) > 2 && any(indatsz(1:end - 1) ~= obj.data_dimensions)) || (ndims(dspan) == 2 && size(dspan,2) ~= obj.data_dimensions)
                error('Tried to add new data to a general_interpolator which did not match the provided dimensions.');
            end
            
            if ndims(obj.data_dimensions) > 2
                dspan = permute(dspan, [ndims(dspan), 1:ndims(dspan)-1]);
            end
            dspan_resize = reshape(dspan, [indatsz(1), prod(indatsz(2:end))]); % Resizing to two dimensions for easy interpolation. 1st dim is always time.
            
            obj.tspan_complete = [obj.tspan_complete; tspan];
            obj.dspan_complete = [obj.dspan_complete; dspan_resize];
        end
        function dat_at_time = get_at_time(obj, tget)
            dat_interp = interp1(obj.tspan_complete, obj.dspan_complete, tget);
            
            if ndims(obj.data_dimensions) > 2
                dat_interp = permute(dat_interp, [2:obj.data_dimensions, 1]); % Put time dimension at the end again.
            else
                dat_at_time = dat_interp;
            end
            dat_at_time = reshape(dat_interp, [1,obj.data_dimensions]);     
        end
    end
end