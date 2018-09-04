classdef IIWAImporter < handle
    properties
        robot;
        ik;
%         iklm
        gik;
        height_constraint;
        position_target;
        home_config;
        base_tform; % Transform for the base/world link of the robot, i.e. actually moves the robot.
        link_patches;
        hgtransforms;
        home_transforms_inverse;
        weights = [0.5 0.5 0.5 1 1 1]; % Angles then positions.
    end
    methods
        function obj = IIWAImporter(fig_handle, varargin)
            obj.robot = importrobot('iiwa14.urdf'); % Import from URDF. Matlab has the file in it's own path. Wow.
            if ~isempty(varargin)
                obj.base_tform = varargin{1};
                obj.robot.Bodies{1}.Joint.setFixedTransform(obj.base_tform);
                
                % If we want a visual mounting platform.
                cyl_radius = 1;
                cyl_height = 3;
                base_loc1 = varargin{1} * [0, 0, 0, 1]';
                base_loc2 = varargin{1} * [0, 0, -cyl_height, 1]';
                [cyl_sides, cyl_lids] = genCylinder(base_loc1(1:3), base_loc2(1:3), cyl_radius, 16);
                cyl_sides.FaceColor = [0.2, 0.2, 0.2];
                cyl_sides.FaceAlpha = 0.75;
                cyl_lids.FaceColor = [0.2, 0.2, 0.2];    
                cyl_lids.FaceAlpha = 0.75;
            else
                obj.base_tform = eye(4);
            end
            
            %% Use MATLAB's robot toolbox to pull stuff in.
            obj.ik = robotics.InverseKinematics('RigidBodyTree', obj.robot);
           obj.ik.SolverParameters.AllowRandomRestart = false;
           obj.ik.SolverParameters.SolutionTolerance = 1e-12;
           obj.ik.SolverParameters.EnforceJointLimits = true;
           
           % I found levenburgmarkqerwsdfs to suck.
%            obj.iklm = robotics.InverseKinematics('RigidBodyTree', obj.robot, 'SolverAlgorithm', 'LevenbergMarquardt');
%            obj.iklm.SolverParameters.SolutionTolerance = 1e-12;
%            obj.iklm.SolverParameters.EnforceJointLimits = true;
%            obj.iklm.SolverParameters.UseErrorDamping = false;
            obj.gik = robotics.GeneralizedInverseKinematics('RigidBodyTree', obj.robot, 'ConstraintInputs', {'cartesian', 'position'});
            
            % Make a constraint for each body which shouldn't be too close
            % to the ground.
            obj.height_constraint = cell(1, length(obj.robot.BodyNames) - 2);
            for i = 3:length(obj.robot.BodyNames)
                obj.height_constraint{i - 2} = robotics.CartesianBounds(obj.robot.BodyNames{i});
                obj.height_constraint{i - 2}.Bounds = [-inf, inf; -inf, inf; 0.01, inf];
            end
            
            obj.position_target = robotics.PositionTarget('iiwa_link_0');
            obj.position_target.TargetPosition([1,1,1]);
            obj.position_target.ReferenceBody = 'iiwa_link_ee';
            obj.position_target.PositionTolerance = 0.01;
            
            
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
        
        function [jnt_angle, solinfo] = single_ik_call(obj, tform, guess, body_part_name)
            [jnt_angle, solinfo] = obj.ik(body_part_name, tform, obj.weights, guess); % Keep using the prev solution as the guess for the next.
        end
        
        function jnt_angle = single_constrained_ik_call(obj, tform, guess, body_part_name)
            obj.position_target.TargetPosition = tform(1:3,4)';
            jnt_angle = obj.gik(guess, obj.height_constraint{end}, obj.position_target); % Keep using the prev solution as the guess for the next.
        end
        
        function [jnt_angles, sol_info] = make_simple_trajectory(obj, tform1, tform2, guess, body_part_name, numpts)
            traj = exampleHelperSE3Trajectory( tform1, tform2, numpts );
            
            jnt_angles = cell(1, numpts);
            sol_info = cell(1, numpts);
            
            for i = 1:size(traj,3)
                [guess, solinfo] = obj.ik(body_part_name, traj(:,:,i), obj.weights, guess); % Keep using the prev solution as the guess for the next.
                jnt_angles{i} = guess;
                sol_info{i} = solinfo;
            end
        end
        
        function [jnt_angles, sol_info] = make_multi_point_trajectory(obj, tform_list, guess, body_part_name, kill_with_bad_sol)
            jnt_angles = cell(1, size(tform_list,3));
            sol_info = cell(1, size(tform_list,3));

            not_bad = true;
            messup_error = 0; % If we get a bad error, everyone else after this point gets it too.
            for i = 1:size(tform_list,3)
                if not_bad
                    [guess, solinfo] = obj.ik(body_part_name, tform_list(:,:,i), obj.weights, guess); % Keep using the prev solution as the guess for the next.
                    jnt_angles{i} = guess;
                    sol_info{i} = solinfo;
                    if kill_with_bad_sol && solinfo.PoseErrorNorm > 1e-5
                        fprintf('Bailed on IK at %d with an error of %f.\n', i, solinfo.PoseErrorNorm);
                        not_bad = false;
                        messup_error = solinfo.PoseErrorNorm;
                    end
                else
                    jnt_angles{i} = guess;
                    solinfo.PoseErrorNorm = messup_error;
                    sol_info{i} = solinfo;
                end
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
