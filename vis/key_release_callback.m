function key_release_callback(src,dat)
global shift_down;

if strcmp(dat.Key, 'shift')
    shift_down = false;
end
end

