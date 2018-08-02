classdef IIWAImporter
    properties
        robot;
        ik;
        home_config;
        link_patches;
        hgtransforms;
        home_transforms_inverse;
        weights = [0.5 0.5 0.5 1 1 1];
    end
    methods
        function obj = IIWAImporter(fig_handle)
            
            obj.robot = importrobot('iiwa14.urdf'); % Import from URDF. Matlab has the file in it's own path. Wow.
            
            %% Use MATLAB's robot toolbox to pull stuff in.
            obj.ik = robotics.InverseKinematics('RigidBodyTree', obj.robot);
            obj.ik.SolverParameters.AllowRandomRestart = false;
            obj.home_config = obj.robot.homeConfiguration; % Straight up home configuration.
            fig_orig = figure(101); % This is just a temporary holder for the imported model. Unfortunately MATLAB doesn't provide the functions I want.
            fig_orig.Visible = 'off';
            ax_orig = show(obj.robot, obj.home_config, 'Frames', 'off'); % Plot this in order to pull the transforms faces/verts.
            
            % Note: all orders made to be from ee -> base.
            robo_links = {}; % Patch objects.
            robo_tforms = {}; % hgtransforms storing the patch objects.
            base_tforms_inv = {}; % Undoes the original home position transformation.
            figure(fig_handle); % Make sure we're putting new shapes into the original figure.
            for i = 1:8 % There are extra patch objects in there which are meaningless to me.
                plot_obj = ax_orig.Children(i);
                if isa(plot_obj, 'matlab.graphics.primitive.Patch')
                    faces = plot_obj.Faces;
                    verts = plot_obj.Vertices;
                    p = patch('Faces',faces,'Vertices',verts);
                    p.LineStyle = 'none';
                    p.CDataMapping = 'Direct';
                    p.FaceColor = plot_obj.FaceColor;
                    p.SpecularStrength = plot_obj.SpecularStrength;
                    p.FaceLighting = plot_obj.FaceLighting;
                    p.DiffuseStrength = plot_obj.DiffuseStrength;
                    
                    hold on;
                    tform = hgtransform; % Assign a transform to each object.
                    p.Parent = tform;
                    robo_tforms{end + 1} = tform;
                    robo_links{end + 1} = p;
                else
                    error('patches in the wrong order somewhere in the iiwa importer.');
                end
                base_tforms_inv{i} = inv(getTransform(obj.robot, obj.home_config, obj.robot.BodyNames{i}));
            end
            
            hold off;
            obj.link_patches = robo_links;
            obj.hgtransforms = robo_tforms;
            obj.home_transforms_inverse = base_tforms_inv;
            
            close(fig_orig);
        end
        
        function tform = getIIWATForm(obj, config, body_part_name)
            tform = getTransform(obj.robot, config, body_part_name);
        end
 
        function jnt_angle = single_ik_call(obj, tform, guess, body_part_name)
            jnt_angle = obj.ik(body_part_name, tform, obj.weights, guess); % Keep using the prev solution as the guess for the next. 
        end
        
        function jnt_angles = make_simple_trajectory(obj, tform1, tform2, guess, body_part_name, numpts)
            traj = exampleHelperSE3Trajectory( tform1, tform2, numpts );
            
            jnt_angles = cell(1,numpts);
            for i = 1:size(traj,3)
                guess = obj.ik(body_part_name, traj(:,:,i), obj.weights, guess); % Keep using the prev solution as the guess for the next.
                jnt_angles{i} = guess;
            end
        end
        
        function jnt_angles = make_multi_point_trajectory(obj, tform_list, guess, body_part_name)
            jnt_angles = cell(1,size(tform_list,3));
            for i = 1:size(tform_list,3)
                guess = obj.ik(body_part_name, tform_list(:,:,i), obj.weights, guess); % Keep using the prev solution as the guess for the next.
                jnt_angles{i} = guess;
            end
        end
        
        function display_at_pose(obj, joint_pos_struct)
            for j = 1:8 % Go from base to ee.
                res_tform = getTransform(obj.robot, joint_pos_struct, obj.robot.BodyNames{j});
                obj.hgtransforms{9 - j}.Matrix = res_tform * obj.home_transforms_inverse{j};
            end
        end
        
        function joint_pos_struct = interpolate_traj(obj, breaks, configs, curr_time)
            spacing = 1:length(breaks);
            idx_in_between = interp1(breaks, spacing, curr_time);
            idx_below = floor(idx_in_between);
            idx_above = ceil(idx_in_between);
            
            if idx_below == idx_above
                joint_pos_struct = configs{idx_below};
                return;
            elseif curr_time < breaks(1)
                joint_pos_struct = configs{1};
                return;
            elseif curr_time > breaks(end)
                joint_pos_struct = configs{end};
                return;
            else
                dt = breaks(idx_above) - breaks(idx_below);
                lower = [configs{idx_below}.JointPosition];
                upper = [configs{idx_above}.JointPosition];
               
                interp_mat = (curr_time - breaks(idx_below))/dt * (upper - lower) + lower; 
                
                joint_pos_struct = configs{1};
                for i = 1:length(interp_mat)
                   joint_pos_struct(i).JointPosition = interp_mat(i);
                end
                return;
            end 
        end
    end
end
