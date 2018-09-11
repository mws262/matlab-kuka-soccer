function key_press_callback( src, dat )
% KEY_PRESS_CALLBACK Function which gets called automatically when
% assigned as a callback for the visualizer. Triggered on a keyboard key
% being pressed down. Handles doing camera orbits if shift is not pressed.
% Arrow keys trigger the motion. Does camera and camera target shifting
% (panning) if shift is pressed. Works closely with key_release_callback.
%
%   KEY_PRESS_CALLBACK(src, dat)
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

global shift_down; % Global so other callbacks can use it, and key_release_callback can listen for it being released.

turn_delta = 0.25; % Multiplier for tilting.
trans_delta = 0.1; % Multiplier for panning.

curr_cam_pos = src.Children.CameraPosition;
curr_cam_tar = src.Children.CameraTarget;
cam_vec = curr_cam_tar - curr_cam_pos;
cam_up = cross(cross(cam_vec, [0, 0, 1]), cam_vec);

shift_down = ~isempty(dat.Modifier) && strcmp(dat.Modifier{1}, 'shift');
switch dat.Key
    case 'uparrow'
        if shift_down % Pan forwards.
            forward_vec = (curr_cam_tar - curr_cam_pos) .* [1 1 0];
            forward_vec = forward_vec/norm(forward_vec);
            src.Children.CameraPosition = curr_cam_pos + trans_delta * forward_vec;
            src.Children.CameraTarget = curr_cam_tar + trans_delta * forward_vec;
        else % Tilt upwards.
            src.Children.CameraPosition = curr_cam_pos + cam_up/(norm(cam_up) + eps) * turn_delta;
        end
    case 'downarrow'
        if shift_down % Pan backwards.
            forward_vec = (curr_cam_tar - curr_cam_pos) .* [1 1 0];
            forward_vec = forward_vec/norm(forward_vec);
            src.Children.CameraPosition = curr_cam_pos - trans_delta * forward_vec;
            src.Children.CameraTarget = curr_cam_tar - trans_delta * forward_vec;
        else % Tilt downwards.
            if curr_cam_pos(3) > 0.2 % No ground-penetrating cameras
                src.Children.CameraPosition = curr_cam_pos - cam_up/(norm(cam_up) + eps) * turn_delta;
            end
        end
    case 'rightarrow'
        if shift_down % Pan right.
            forward_vec = (curr_cam_tar - curr_cam_pos) .* [1 1 0];
            forward_vec = forward_vec/norm(forward_vec);
            right_vec = cross(forward_vec, [0, 0, 1]);
            src.Children.CameraPosition = curr_cam_pos + trans_delta * right_vec;
            src.Children.CameraTarget = curr_cam_tar + trans_delta * right_vec;
        else % Tilt right.
            cam_r = cross(cam_vec, cam_up);
            src.Children.CameraPosition = curr_cam_pos + cam_r/(norm(cam_r) + eps) * turn_delta;
        end
    case 'leftarrow'
        if shift_down % Pan left.
            forward_vec = (curr_cam_tar - curr_cam_pos) .* [1 1 0];
            forward_vec = forward_vec/norm(forward_vec);
            right_vec = cross(forward_vec, [0, 0, 1]);
            src.Children.CameraPosition = curr_cam_pos - trans_delta * right_vec;
            src.Children.CameraTarget = curr_cam_tar - trans_delta * right_vec;
        else % Tilt left.
            cam_r = cross(cam_vec, cam_up);
            src.Children.CameraPosition = curr_cam_pos - cam_r/(norm(cam_r) + eps) * turn_delta;
        end
    otherwise
        % Other events maybe added in the future.
end
end
