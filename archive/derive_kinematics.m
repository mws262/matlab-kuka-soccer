clear all; close all;
lengths = sym('l',[7,1], 'positive');
widths = sym('w',[7,1], 'positive');
heights = sym('h',[7,1], 'positive');
joint_locs = sym('jloc',[7,3], 'real');
th = sym('th', [7,1], 'real'); % Joint angles. 0s being straight up.
thd = sym('thd', [7,1], 'real');
thdd = sym('thdd', [7,1], 'real');
ij = sym('ij',[7,1], 'real'); % Unit vectors for frames associated with links.
jj = sym('jj',[7,1], 'real');
kj = sym('kj',[7,1], 'real');
unitvecs = [ij, jj, kj]';

% X,Y,Z unit vectors according to the world.
i = [1 0 0]';
j = [0 1 0]';
k = [0 0 1]';
world_unit = [i,j,k];

xrot = @(angle)([1 0 0; 0 cos(angle) -sin(angle); 0 sin(angle) cos(angle)]);
yrot = @(angle)([cos(angle) 0 sin(angle); 0 1 0; -sin(angle) 0 cos(angle)]);
zrot = @(angle)([cos(angle) -sin(angle) 0; sin(angle) cos(angle) 0; 0 0 1]);

% link 0 is the ground.

% Joint rotations relative to their local frams.
R_1_0 = zrot(th(1));
R_2_1 = xrot(th(2));
R_3_2 = zrot(th(3));
R_4_3 = xrot(th(4));
R_5_4 = zrot(th(5));
R_6_5 = xrot(th(6));
R_7_6 = zrot(th(7));

% Unit vectors of all frames transformed.
u1 = R_1_0*xrot(pi/2)*world_unit;
u2 = R_2_1*u1;
u3 = R_3_2*u2;
u4 = R_4_3*u3;
u5 = R_5_4*u4;
u6 = R_6_5*u5;
u7 = R_7_6*u6;

% Joint positions relative to base in world frame.
r_jnext_jprev(:,1) = lengths(1) .* u1(:,1) + widths(1) .* u1(:,2) + heights(1) .* u1(:,3);
r_jnext_jprev(:,2) = lengths(2) .* u2(:,2) + widths(2) .* u2(:,2) + heights(2) .* u2(:,3);
r_jnext_jprev(:,3) = lengths(3) .* u3(:,2) + widths(2) .* u3(:,2) + heights(3) .* u3(:,3);
r_jnext_jprev(:,4) = lengths(4) .* u4(:,2) + widths(2) .* u4(:,2) + heights(4) .* u4(:,3);
r_jnext_jprev(:,5) = lengths(5) .* u5(:,2) + widths(2) .* u5(:,2) + heights(5) .* u5(:,3);
r_jnext_jprev(:,6) = lengths(6) .* u6(:,2) + widths(2) .* u6(:,2) + heights(6) .* u6(:,3);
r_jnext_jprev(:,7) = lengths(7) .* u7(:,2) + widths(2) .* u7(:,2) + heights(7) .* u7(:,3);

r_jnext_base = cumsum(r_jnext_jprev,2);

% Dumb dimension assignment for now.
r_jnext_base_num = subs(r_jnext_base, lengths, ones(size(lengths)));
r_jnext_base_num = subs(r_jnext_base_num, widths, zeros(size(lengths)));
r_jnext_base_num = subs(r_jnext_base_num, heights, zeros(size(lengths)));

jpos_fcn = matlabFunction(r_jnext_base_num);


fig = figure;
fig.Position = [200, 200, 1000, 1000];
arm_plot = plot(0,0, 'MarkerSize', 20, 'LineWidth', 3);
arm_plot.Marker = '.';
axis([-7 7 -7 7 0 7]);
ax = fig.Children;
ax.Clipping = 'off';
daspect([1,1,1]);



poses = [0 0 0 0 0 0 0;
    fliplr(tril(ones(7)))];
times = linspace(0, 10, size(poses,1));

prev_time = 0;
curr_time = 0;
tic;
while curr_time < times(end)  
    curr_angs = interp1(times, poses, curr_time);
    curr_pos = jpos_fcn(curr_angs(1), curr_angs(2), curr_angs(3), curr_angs(4), curr_angs(5), curr_angs(6), curr_angs(7));
    arm_plot.XData = curr_pos(1,:);
    arm_plot.YData = curr_pos(2,:);
    arm_plot.ZData = curr_pos(3,:);
    
    drawnow;
    prev_time = curr_time;
    curr_time = toc;
end

