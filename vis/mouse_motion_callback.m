function mouse_motion_callback(src, dat)
% MOUSE_MOTION_CALLBACK Function which gets called automatically when
% assigned as a callback for the visualizer. Triggered on mouse motion.
% This callback is turned off when mouse buttons are not pressed. When
% mouse is dragged, orbits camera around the camera target. If shift is
% pressed, translates both camera and camera target relative to the plane
% of the ground.
%
%   MOUSE_MOTION_CALLBACK(src, dat)
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
global shift_down;

if isempty(previous_pos)
    previous_pos = src.CurrentPoint;
end
current_pos = src.CurrentPoint; % Capture current mouse position in screen coords.
mouse_diff = current_pos - previous_pos;
mouse_diffX = mouse_diff(1);
mouse_diffY = mouse_diff(2);
camvec = src.Children.CameraPosition - src.Children.CameraTarget;
if shift_down
    forwardvec = camvec .* [1,1,0];
    forwardvec = forwardvec / norm(forwardvec);
    rightvec = cross(camvec, [0, 0, 1]);
    forwarddiff = 0.003 * mouse_diffY * forwardvec;
    rightdiff = 0.001 * mouse_diffX * rightvec;
    src.Children.CameraTarget = src.Children.CameraTarget + forwarddiff + rightdiff;
    src.Children.CameraPosition = src.Children.CameraPosition + forwarddiff + rightdiff;
else
    src.Children.CameraPosition = (angle2dcm(0, 0, 0.006*mouse_diffX, 'xyz') * camvec')' + src.Children.CameraTarget;
    camvec = src.Children.CameraPosition - src.Children.CameraTarget;
    sidevec = cross([0,0,1], camvec);
    if mouse_diffY > 0 || dot(camvec, camvec) - camvec(3)^2 > 0.2 % Avoid getting in gimbal lock range.
        twirl_rot = axang2rotm([sidevec, mouse_diffY * 0.003]);
        src.Children.CameraPosition = (twirl_rot * camvec')' + src.Children.CameraTarget;
    end
    if src.Children.CameraPosition(3) < 0.1
        src.Children.CameraPosition(3) = 0.1;
    end
end
previous_pos = current_pos;
end

