classdef general_interpolator < handle
    % GENERAL_INTERPOLATOR Expandable data storage container which can return
    % an interpolated value at requested times.
    %   Data can be added again and again as long as the times are
    %   monotonically increasing. If the data is one-dimensional, then the
    %   first dimension should be time. If it is higher dimensional (e.g.
    %   rotation matrices), then the last dimension should be time.
    %
    %   Methods:
    %       general_interpolator(data_dimensions) -- Constructor.
    %       add_to_end_at_time(obj, tspan, dspan) -- Adds more data to this
    %       interpolator. Must be greater in time than previous data.
    %       get_at_time(obj, tget) -- Get an interpolated value at a specified
    %       time.
    %   Properties:
    %       `dspan_complete` -- All data being stored. Note that for 2D data
    %       (like a 3x3 rotation matrix), this data is resized to be stored so
    %       each row is a single time-element of the data.
    %       `tspan_complete` -- All time elements being stored. Will correspond
    %       to all data in `dspan_complete`.
    %       `data_dimensions` -- Dimension of each time-element of the data
    %       being stored. This is defined when creating this
    %       general_interpolator.
    %
    %   See also GENERAL_INTERPOLATOR.GENERAL_INTERPOLATOR,
    %   GENERAL_INTERPOLATOR.ADD_TO_END_AT_TIME,
    %   GENERAL_INTERPOLATOR.GET_AT_TIME.
    %
    
    properties
        dspan_complete;
        tspan_complete;
        data_dimensions;
    end
    methods
        function obj = general_interpolator(data_dimensions)
            % GENERAL_INTERPOLATOR Make a new interpolator object.
            %
            %   obj = GENERAL_INTERPOLATOR(data_dimensions)
            %
            %   Inputs:
            %       `data_dimensions` -- Size of each element of the data
            %       to be interpolated/stored.
            %   Outputs:
            %       `obj` -- A new general_interpolator object.
            %
            
            validateattributes(data_dimensions, {'numeric'}, {'integer', 'positive'});
            obj.data_dimensions = data_dimensions;
        end
        
        function add_to_end_at_time(obj, tspan, dspan)
            % ADD_TO_END_AT_TIME Add additional time elements to this
            % interpolator object. This data must come after existing data.
            %
            %   ADD_TO_END_AT_TIME(obj, tspan, dspan)
            %   obj.ADD_TO_END_AT_TIME(tspan, dspan)
            %
            %   Inputs:
            %       `obj` -- Existing general_interpolator object.
            %       `tspan` -- Additional time elements. Must be
            %       monotonically increasing and greater than all existing
            %       times in this object.
            %       `dspan` -- Additional data elements corresponding to
            %       `tspan`. If each time-element is a vector, then the
            %       first dimension is along time. If each element is a
            %       matrix, then the last dimension should be along time.
            %           e.g. First time element of rotation matrix
            %           dspan(:,:,1). First time element of 3D position
            %           dspan(1,:).
            %
            
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
            % GET_AT_TIME Get data stored in this object interpolated to a
            % specified time.
            %
            %   dat_at_time = GET_AT_TIME(obj, tget)
            %   dat_at_time = obj.GET_AT_TIME(tget)
            %
            %   Inputs:
            %       `obj` -- general_interpolator object being queried.
            %       `tget` -- Time at which to interpolate the data.
            %   Outputs:
            %       `dat_at_time` -- Data interpolated to time `tget`. Will
            %       have dimension defined when creating this
            %       general_interpolator.
            %
            
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
    end
    methods (Access = protected)
        % Same as build-in ndims, but can return 1 if there is only 1
        % non-singleton dimension. Will not return less than 1.
        function dims = ndims_with_one(obj, arr)
            dims = max(length(arr(arr ~= 1)),1);
        end
    end
end