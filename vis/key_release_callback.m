function key_release_callback(src,dat)
% KEY_RELEASE_CALLBACK Function which gets called automatically when
% assigned as a callback for the visualizer. Triggered on a keyboard key
% being released. Currently only listens for modifier keys being released.
%
%   KEY_RELEASE_CALLBACK(src, dat)
%
%   Inputs:
%       `src` -- Which graphics object triggered this callback.
%       `dat` -- Data structure with information about the keyboard event.
%
%   Outputs: <none>
%
%   See also MOUSE_MOTION_CALLBACK, MOUSE_UP_CALLBACK, MOUSE_DOWN_CALLBACK,
%   KEY_CALLBACK, KEY_RELEASE_CALLBACK, PATCHFACEFCN, MAKE_VISUALIZER_SCENE.
%

global shift_down;

if strcmp(dat.Key, 'shift')
    shift_down = false;
end
end

