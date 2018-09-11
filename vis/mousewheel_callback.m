function mousewheel_callback( src, dat )
% MOUSEWHEEL_CALLBACK Function which gets called automatically when
% assigned as a callback for the visualizer. Triggered on mouse scroll
% wheel. Handles zooming in/out in the direction of the camera target along
% the line between the camera and the camera target.
%
%   MOUSEWHEEL_CALLBACK(src, dat)
%
%   Inputs:
%       `src` -- Which graphics object triggered this callback.
%       `dat` -- Data structure with information about the mouse wheel
%       event.
%
%   Outputs: <none>
%
%   See also MOUSE_MOTION_CALLBACK, MOUSE_UP_CALLBACK, MOUSE_DOWN_CALLBACK,
%   KEY_CALLBACK, KEY_RELEASE_CALLBACK, PATCHFACEFCN, MAKE_VISUALIZER_SCENE.
%

delta = 0.25; % Multiplier for zooming. Hand-tuned.

curr_cam_pos = src.Children.CameraPosition;
curr_cam_tar = src.Children.CameraTarget;

cam_vec = curr_cam_tar - curr_cam_pos;

if dat.VerticalScrollCount > 0 % Zoom out
    src.Children.CameraPosition = curr_cam_pos - cam_vec/norm(cam_vec)*delta;
elseif dat.VerticalScrollCount < 0 && dot(cam_vec, cam_vec) > 1 % Zoom in, but only if we aren't too close to the target.
    src.Children.CameraPosition = curr_cam_pos + cam_vec/norm(cam_vec)*delta;
end
end

