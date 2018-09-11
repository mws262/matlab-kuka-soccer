function mouse_up_callback(src,dat)
% MOUSE_UP_CALLBACK Function which gets called automatically when
% assigned as a callback for the visualizer. Triggered on mouse buttons
% being released. This terminates any dragging interactions which might be
% occuring by unassigning the WindowButtonMotionFcn.
%
%   MOUSE_UP_CALLBACK(src, dat)
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

if strcmp(dat.EventName, 'WindowMouseRelease')
    src.WindowButtonMotionFcn = '';
end
end

