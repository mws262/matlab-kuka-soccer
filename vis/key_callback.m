function key_callback( src, dat )
%KEY_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
turn_delta = 0.25;
trans_delta = 0.1;

curr_cam_pos = src.Children.CameraPosition;
curr_cam_tar = src.Children.CameraTarget;
cam_vec = curr_cam_tar - curr_cam_pos;
cam_up = cross(cross(cam_vec, [0, 0, 1]), cam_vec);
switch dat.Key
    case 'uparrow'
        if ~isempty(dat.Modifier) && strcmp(dat.Modifier{1}, 'shift')
            forward_vec = (curr_cam_tar - curr_cam_pos) .* [1 1 0];
            forward_vec = forward_vec/norm(forward_vec);
            src.Children.CameraPosition = curr_cam_pos + trans_delta * forward_vec;
            src.Children.CameraTarget = curr_cam_tar + trans_delta * forward_vec;
        else
            src.Children.CameraPosition = curr_cam_pos + cam_up/(norm(cam_up) + eps) * turn_delta;
        end
    case 'downarrow'
        if ~isempty(dat.Modifier) && strcmp(dat.Modifier{1}, 'shift')
            forward_vec = (curr_cam_tar - curr_cam_pos) .* [1 1 0];
            forward_vec = forward_vec/norm(forward_vec);
            src.Children.CameraPosition = curr_cam_pos - trans_delta * forward_vec;
            src.Children.CameraTarget = curr_cam_tar - trans_delta * forward_vec;         
        else
            if curr_cam_pos(3) > 0.2 % No ground-penetrating cameras
                src.Children.CameraPosition = curr_cam_pos - cam_up/(norm(cam_up) + eps) * turn_delta;
            end
        end
    case 'rightarrow'
        if ~isempty(dat.Modifier) && strcmp(dat.Modifier{1}, 'shift')
            forward_vec = (curr_cam_tar - curr_cam_pos) .* [1 1 0];
            forward_vec = forward_vec/norm(forward_vec);
            right_vec = cross(forward_vec, [0, 0, 1]);
            src.Children.CameraPosition = curr_cam_pos + trans_delta * right_vec;
            src.Children.CameraTarget = curr_cam_tar + trans_delta * right_vec;
        else
            cam_r = cross(cam_vec, cam_up);
            src.Children.CameraPosition = curr_cam_pos + cam_r/(norm(cam_r) + eps) * turn_delta;
        end
    case 'leftarrow'
        if ~isempty(dat.Modifier) && strcmp(dat.Modifier{1}, 'shift')
            forward_vec = (curr_cam_tar - curr_cam_pos) .* [1 1 0];
            forward_vec = forward_vec/norm(forward_vec);
            right_vec = cross(forward_vec, [0, 0, 1]);
            src.Children.CameraPosition = curr_cam_pos - trans_delta * right_vec;
            src.Children.CameraTarget = curr_cam_tar - trans_delta * right_vec;
        else
            cam_r = cross(cam_vec, cam_up);
            src.Children.CameraPosition = curr_cam_pos - cam_r/(norm(cam_r) + eps) * turn_delta;
        end
    otherwise
        
end


end

