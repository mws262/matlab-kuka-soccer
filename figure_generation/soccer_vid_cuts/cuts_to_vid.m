% Read in video - taken from https://www.youtube.com/watch?v=bQT3MP5Moq8&t=259s
% Cut a bunch of clips together in a brief array of videos displayed at
% once. Several versions of vid -- array, big sequence, and cut sequence.
vr = VideoReader('soccer_footwork_vid.mp4');
fr = vr.FrameRate;

%% Clip timings
toe_pull = [72, 90]; % Start/stop times.
cryuff = [113, 140];
roll_over = [159, 180];
drag_back = [229, 252];
hop_step = [270, 298];
l_cut = [317, 336];
behind_the_back = [353, 377];
body_fient = [392, 415];

all_timings = [toe_pull; cryuff; roll_over; drag_back; hop_step; l_cut; behind_the_back; body_fient];
min_duration = min(range(all_timings, 2)); % Reduce each clip to the size of the smallest in the lot.
all_timings(:,2) = all_timings(:,1) + min_duration;
num_frames = ceil(min_duration * fr);

%% Read in video.
captured_frames = zeros(vr.Height, vr.Width, 3, num_frames, size(all_timings,1), 'uint8');
for j = 1:size(all_timings)
    vr.CurrentTime = all_timings(j,1);
    for i = 1:num_frames
        captured_frames(:,:,:,i,j) = vr.readFrame();
    end
end
gap_size = 20;
reduction_factor = 0.4;

%% All in a big 2x4 array at the same time.
vw = VideoWriter(['./soccer_array.mp4'], 'MPEG-4');
vw.open();
for i = 1:size(captured_frames,4)
    frm_concat = [captured_frames(:,:,:,i,1), zeros(size(captured_frames,1), gap_size, 3, 1, 'uint8'), ...
        captured_frames(:,:,:,i,2), zeros(size(captured_frames,1), gap_size, 3, 1, 'uint8'), ...
        captured_frames(:,:,:,i,3), zeros(size(captured_frames,1), gap_size, 3, 1, 'uint8'), ...
        captured_frames(:,:,:,i,4); ...
        zeros(gap_size, size(captured_frames,2) * 4 + gap_size * 3, 3, 1, 'uint8'); ...
        captured_frames(:,:,:,i,5), zeros(size(captured_frames,1), gap_size, 3, 1, 'uint8'), ...
        captured_frames(:,:,:,i,6), zeros(size(captured_frames,1), gap_size, 3, 1, 'uint8'), ...
        captured_frames(:,:,:,i,7), zeros(size(captured_frames,1), gap_size, 3, 1, 'uint8'), ...
        captured_frames(:,:,:,i,8)];
    frm_concat_compress = imresize(frm_concat, reduction_factor);
    fprintf('Frame: %d\n', i);
    vw.writeVideo(frm_concat_compress);
end
vw.close();

%% In a total sequence.
vw = VideoWriter(['./soccer_sequence.mp4'], 'MPEG-4');
vw.open();
for j = 1:size(captured_frames, 5) % Iterate through clips.
    for i = 1:size(captured_frames,4) % Iterate through video frames.
        frm_concat = captured_frames(:,:,:,i,j);
        frm_concat_compress = imresize(frm_concat, reduction_factor);
        fprintf('Frame: %d\n', i);
        vw.writeVideo(frm_concat_compress);
    end
end
vw.close();

%% Just the slomo parts.
vw = VideoWriter(['./soccer_sequence_slomo.mp4'], 'MPEG-4');
clip_duration_each = 10;
frame_duration_each = clip_duration_each * fr;
fade_duration = 1;
fade_in_fr = ceil(fr * fade_duration);
fade_out_fr = floor(frame_duration_each - fr * fade_duration);
vw.open();
for j = 1:size(captured_frames, 5) % Iterate through clips.
    for i = 1:frame_duration_each % Iterate through video frames.
        frm_concat = captured_frames(:,:,:,i,j);
        frm_concat_compress = imresize(frm_concat, reduction_factor);

        % Add fade-in / fade-out. It's too jarring without.
        if i < fade_in_fr
            frm_concat_compress = frm_concat_compress * (i / fade_in_fr);
        elseif i > fade_out_fr
            frm_concat_compress = frm_concat_compress * (i / fade_out_fr);
        end
        fprintf('Frame: %d\n', i);
        vw.writeVideo(frm_concat_compress);
    end
end
vw.close();

