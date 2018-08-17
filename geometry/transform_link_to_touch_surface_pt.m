function net_transform = transform_link_to_touch_surface_pt(link_surface_pt, link_surface_normal, goal_pt, goal_normal, scrub_angle)
%% Align a link with a point in the world. Make the normal point in a specified direction.
% Inputs:
% link_surface_pt -- Point on the surface (before transformed) that we wish
% to align with some world point.
% link_surface_normal -- Original, untransformed normal on the surface.
% goal_pt -- World point in space we wish to align the link's surface to.
% goal_normal -- Target normal to be aligned with link_surface_normal.
% scrub_angle -- All others define 5 DOF. This defines the 6th, which is
% spin about the defined normal vector.
%
% Returns:
% net_transform -- 4x4 transform from original position to goal.

%% Translational components.
% Point on surface (i.e. contact pt) that we want to rotate about.
surface_to_origin_translation = trvec2tform(-link_surface_pt);
% Goal point in world space.
world_origin_to_goal_pt_translation = trvec2tform(goal_pt);

%% Rotational components.
surface_normal_to_goal_normal_rotation = rotm2tform(get_rotation_from_vecs(link_surface_normal, goal_normal)); % Defines all but 1 rotational DOF.
scrub_spin_rotation = rotm2tform(axang2rotm([goal_normal, scrub_angle])); % Spins the link, keeping the point collocated with the goal point and still aligned with the normal vector.sc

net_transform = world_origin_to_goal_pt_translation * scrub_spin_rotation * surface_normal_to_goal_normal_rotation * surface_to_origin_translation;

end