function mouse_down_callback(src,dat)
global previous_pos;
if strcmp(dat.EventName, 'WindowMousePress')
    previous_pos = src.CurrentPoint;
    src.WindowButtonMotionFcn = @mouse_motion_callback;
end
end

