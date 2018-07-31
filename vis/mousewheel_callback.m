function [ output_args ] = mousewheel_callback( src, dat )
%MOUSEWHEEL_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
delta = 0.25;

curr_cam_pos = src.Children.CameraPosition;
curr_cam_tar = src.Children.CameraTarget;
cam_vec = curr_cam_tar - curr_cam_pos;
if dat.VerticalScrollCount > 0
    src.Children.CameraPosition = curr_cam_pos - cam_vec/norm(cam_vec)*delta;
else
    src.Children.CameraPosition = curr_cam_pos + cam_vec/norm(cam_vec)*delta;
end
end

