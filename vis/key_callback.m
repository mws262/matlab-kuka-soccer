function key_callback( src, dat )
%KEY_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
delta = 0.15;

curr_cam_pos = src.Children.CameraPosition;
curr_cam_tar = src.Children.CameraTarget;
cam_vec = curr_cam_tar - curr_cam_pos;
cam_up = cross(cross(cam_vec, [0, 0, 1]), cam_vec);
switch dat.Key
    case 'uparrow'
        src.Children.CameraPosition = curr_cam_pos + cam_up/(norm(cam_up) + eps) * delta;
    case 'downarrow'
        if curr_cam_pos(3) > 0.2 % No ground-penetrating cameras
            src.Children.CameraPosition = curr_cam_pos - cam_up/(norm(cam_up) + eps) * delta;
        end
    case 'rightarrow'
        cam_r = cross(cam_vec, cam_up);
        src.Children.CameraPosition = curr_cam_pos - cam_r/(norm(cam_r) + eps) * delta;
    case 'leftarrow'
        cam_r = cross(cam_vec, cam_up);
        src.Children.CameraPosition = curr_cam_pos + cam_r/(norm(cam_r) + eps) * delta;
    otherwise
        
end


end

