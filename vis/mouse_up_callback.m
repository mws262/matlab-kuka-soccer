function mouse_up_callback(src,dat)
if strcmp(dat.EventName, 'WindowMouseRelease')
    src.WindowButtonMotionFcn = '';
end
end

