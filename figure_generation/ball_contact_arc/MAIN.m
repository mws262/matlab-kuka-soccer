close all; clear all;
addpath ../../;
addpath ../../vis/;
addpath ../../geometry/;
addpath ../../derived_autogen/;
addpath ../../data/;
addpath ../../path_optim/;
addpath ../../dynamics/;
addpath ../../iiwa_kinematics/;
addpath ../../util/;

scene_fig = make_visualizer_scene();

bpatch = make_ball(0.1);
bpatch.FaceColor = [0.7, 0.7, 0.7];

path_pp = get_path_pp('large_arc', 5);

teval = linspace(min(path_pp.breaks), max(path_pp.breaks), 200);

path_eval = ppval(path_pp, teval)';
accel_pp = fnder(path_pp,2);

accel_eval = ppval(accel_pp, teval)';


draw_path_and_accel(path_eval, accel_eval, 5)

bpatch.Vertices = bpatch.Vertices + path_eval(100,:);

tarc = transpose(-0.5:0.1:pi/2-0.1);
xarc = 0.1*[cos(tarc), zeros(size(tarc)), sin(tarc)] + path_eval(100,:) + [0 0 0.1];
arcpl = plot3(xarc(:,1), xarc(:,2), xarc(:,3), 'g', 'LineWidth', 4);
xarc_off = 0.2*[cos(tarc), zeros(size(tarc)), sin(tarc)]./repmat(cos(tarc), [1,3]);

quiv = quiver3(xarc_off(:,1) +  xarc(:,1), xarc_off(:,2) +  xarc(:,2), xarc_off(:,3) +  xarc(:,3), -xarc_off(:,1), -xarc_off(:,2), -xarc_off(:,3), 'b', 'LineWidth', 1.5);
quiv.AutoScale = 'off'
quiv.MaxHeadSize = 0.1
camtarget([0.9924 0.0169 0]);
campos([0.8097   -0.5762    0.6518]);

save2pdf('../../data/images/ball_force_vecs.pdf', scene_fig, 600);