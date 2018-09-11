function mouse_down_callback(src,dat)
% MOUSE_DOWN_CALLBACK Function which gets called automatically when
% assigned as a callback for the visualizer. Triggered on mouse buttons
% being pressed. Starts listening for dragging motion by assigning
% MOUSE_MOTION_CALLBACK as a mouse motion callback.
%
%   MOUSE_DOWN_CALLBACK(src, dat)
%
%   Inputs:
%       `src` -- Which graphics object triggered this callback.
%       `dat` -- Data structure with information about the mouse event.
%
%   Outputs: <none>
%
%   See also MOUSE_MOTION_CALLBACK, MOUSE_UP_CALLBACK, MOUSE_DOWN_CALLBACK,
%   KEY_CALLBACK, KEY_RELEASE_CALLBACK, PATCHFACEFCN, MAKE_VISUALIZER_SCENE.
%

global previous_pos;
if strcmp(dat.EventName, 'WindowMousePress')
    previous_pos = src.CurrentPoint;
    src.WindowButtonMotionFcn = @mouse_motion_callback;
end
end

