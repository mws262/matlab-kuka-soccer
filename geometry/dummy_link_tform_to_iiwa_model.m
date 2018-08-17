function tform_iiwa = dummy_link_tform_to_iiwa_model(dummy_rot, dummy_trans, ball_contact_shift_tform, contact_location)
%DUMMY_LINK_TFORM_TO_IIWA_MODEL Take a transform which has been found using
%the standalone link and turn it into a transform which can be used on the
%RigidBodyTree version of the iiwa.
% ball_contact_shift_tform reflects that as the arm changes relative
% position on the ball, its normal needs to change too.

% This transforms the graphical version of the free-floating foot for
% planning onto the graphical version of the foot on the full robot. This
% was partially hand-tuned.
% Note that there is another transform which goes between the real version
% of the link in the RigidBodyTree and the graphical version.
% 
% iiwa_link_6_init_tform =    [-1.0000    0.0000   -0.0000    0.0000
%     -0.0000    0.0000    1.0000    0.0000
%     0.0000    1.0000   -0.0000    1.1800
%     0         0         0    1.0000];
% 
% tform_dummy_to_robot =    ([ 0.0000    0.0000    1.0000   -0.2155;
%     0.0000   -1.0000    0.0000    0.0000;
%     1.0000    0.0000   -0.0000    1.1800;
%     0         0         0    1.0000]);
% 
% Might as well precalculate tform_dummy_to_robot \ iiwa_link_6_init_tform:

total_dummy_to_robot =          [0    1.0000         0         0
         0         0   -1.0000         0
   -1.0000         0         0    0.2155
         0         0         0    1.0000];

% tform_dummy(1:3,4) = -tform_dummy(1:3,4);
tform_dummy_trans = trvec2tform(-dummy_trans);
tform_dummy_rot = rotm2tform(dummy_rot');

if length(contact_location) == 3 % Just a vector.
    tform_iiwa =  trvec2tform(contact_location) * ...
        ball_contact_shift_tform * tform_dummy_rot * tform_dummy_trans * total_dummy_to_robot;
else % Already a 4x4 tform.
    tform_iiwa =  contact_location * ...
        ball_contact_shift_tform * tform_dummy_rot * tform_dummy_trans * total_dummy_to_robot;
end
end

