function mesh_integration_test
close all;
clear all;
fig = figure;
iiwa = IIWAImporter(fig);
lpatch = iiwa.link_patches{5};

fig2 = figure;
ax = axes;
lpatch.Parent = ax;
lpatch.FaceAlpha = 0.3;
% [sphere_x,sphere_y,sphere_z] = sphere(4);
% ball_patch = patch(surf2patch(sphere_x, sphere_y, sphere_z, 'triangles'))
% ball_patch.FaceColor = 'b'
% ball_patch.FaceAlpha = 0.3
camlight HEADLIGHT;
view(3);
daspect([1,1,1])
xlabel('x');
ylabel('y');
zlabel('z');

faces = lpatch.Faces;
verts = lpatch.Vertices;

dt = 0.0001;
current_face = 3000;
start_face_verts = verts(faces(current_face,:), :);
current_pt = mean(start_face_verts,1); % sstart at a consistent point on a mesh.
current_normal = get_face_normal(current_face, faces, verts);
vel = mean(start_face_verts(1:2,:,1)) - current_pt + -(start_face_verts(1,:) - start_face_verts(2,:))*0.05;
current_tform = eye(3);
hold on;
plot3(current_pt(1), current_pt(2), current_pt(3), '.r', 'MarkerSize', 20);
for i = 1:10000
    ptstar = current_pt + (current_tform*(vel)')'*dt; % point if we stay on this face. Need to check if we've left it.
    [intersect_pt, remainder, crossing_idx1, crossing_idx2] = find_intersection(current_face, faces, verts, current_pt, ptstar);
    
    if (crossing_idx1 ~= 0) % We have crossed over triangles.
        [neighbor_face, neighbor_idx1, neighbor_idx2] = locate_adjacent(current_face, crossing_idx1, crossing_idx2, faces, verts);
        plot3(intersect_pt(1), intersect_pt(2), intersect_pt(3), '.g', 'MarkerSize', 20);
        
        new_normal = get_face_normal(neighbor_face, faces, verts);
        assert(dot(new_normal, current_normal) >= 0);
        edge_rotation = get_rotation_from_vecs(current_normal, new_normal);
        current_tform = edge_rotation*current_tform;
        current_pt = (edge_rotation*remainder')' + intersect_pt;
        current_face = neighbor_face;
        current_normal = new_normal;
    else
        current_pt = ptstar;
    end
    if (mod(i,200) == 199)
        plot3(current_pt(1), current_pt(2), current_pt(3), '.r', 'MarkerSize', 20);
        drawnow;
    end
    
end
disp('done');
    function normvec = get_face_normal(face, faces, verts)
        pt1 = verts(faces(face,1),:);
        pt2 = verts(faces(face,2),:);
        pt3 = verts(faces(face,3),:);
        normvec = cross(pt2 - pt1, pt3 - pt1);
    end

    function rotation = get_rotation_from_vecs(v1, v2)
        v1 = v1./norm(v1);
        v2 = v2./norm(v2);
        
        v = cross(v1, v2);
        sk = skew(v);
        c = dot(v1, v2);
        s = norm(v);
        rotation = eye(3) + sk + sk^2*(1 - c)/s^2;
        if any(any(isnan(rotation)))
            error('Rotation matrix maker got some NaNs all up in it.');
        end
    end

    function skewmat = skew(vec)
        skewmat =[0 -vec(3) vec(2) ; vec(3) 0 -vec(1) ; -vec(2) vec(1) 0 ];
    end

    function [intersect_pt, remainder, crossing_idx1, crossing_idx2] = find_intersection(face, faces, verts, pt, ptstar)
        p1 = verts(faces(face,1), :);
        p2 = verts(faces(face,2), :);
        p3 = verts(faces(face,3), :);
        
        p21 = p2 - p1;
        p31 = p3 - p1;
        p32 = p3 - p2;
        ptdelta = ptstar - pt;
        
        %         normal = cross(p21, p31);
        %         normal_21 = cross(p21, normal);
        %         normal_31 = cross(-p31, normal);
        %         normal_32 = cross(p32, normal);
        
        %         A = zeros(9);
        %         A(1:3, 1) = ptdelta';
        %         A(1:3, 2) = -p21';
        %         A(4:6, 3) = ptdelta';
        %         A(4:6, 4) = -p31';
        %         A(7:9, 5) = ptdelta';
        %         A(7:9, 6) = -p32';
        %         p1r = p1 - pt;
        %         p2r = p2 - pt;
        %         B = [p1r';p1r';p2r'];
        
        A = zeros(6);
        A(1:2, 1) = ptdelta(1:2)';
        A(1:2, 2) = -p21(1:2)';
        A(3:4, 3) = ptdelta(1:2)';
        A(3:4, 4) = -p31(1:2)';
        A(5:6, 5) = ptdelta(1:2)';
        A(5:6, 6) = -p32(1:2)';
        p1r = p1 - pt;
        p2r = p2 - pt;
        B = [p1r(1:2)';p1r(1:2)';p2r(1:2)'];
        
        cond = rcond(A);
        if ( isnan(cond) || cond < 1e-6)
            A = zeros(6);
            A(1:2, 1) = ptdelta(2:3)';
            A(1:2, 2) = -p21(2:3)';
            A(3:4, 3) = ptdelta(2:3)';
            A(3:4, 4) = -p31(2:3)';
            A(5:6, 5) = ptdelta(2:3)';
            A(5:6, 6) = -p32(2:3)';
            B = [p1r(2:3)';p1r(2:3)';p2r(2:3)'];
            
            cond = rcond(A);
            if ( isnan(cond) || cond < 1e-6)
                A = zeros(6);
                A(1:2, 1) = ptdelta([1,3])';
                A(1:2, 2) = -p21([1,3])';
                A(3:4, 3) = ptdelta([1,3])';
                A(3:4, 4) = -p31([1,3])';
                A(5:6, 5) = ptdelta([1,3])';
                A(5:6, 6) = -p32([1,3])';
                B = [p1r([1,3])';p1r([1,3])';p2r([1,3])'];
            end
        end
        
        tsol = A\B;
        
        if tsol(1) > 0 && tsol(1) < 1 && tsol(2) > 0 && tsol(2) < 1 % Has crossed edge 21
            intersect_pt = tsol(1)*ptdelta + pt;
            remainder = ptstar - intersect_pt;
            crossing_idx1 = 1;
            crossing_idx2 = 2;
            
        elseif tsol(3) > 0 && tsol(3) < 1 && tsol(4) > 0 && tsol(4) < 1 % Has crossed edge 31
            intersect_pt = tsol(3)*ptdelta + pt;
            remainder = ptstar - intersect_pt;
            crossing_idx1 = 1;
            crossing_idx2 = 3;
            
        elseif tsol(5) > 0 && tsol(5) < 1 && tsol(6) > 0 && tsol(6) < 1 % Has crossed edge 32
            intersect_pt = tsol(5)*ptdelta + pt;
            remainder = ptstar - intersect_pt;
            crossing_idx1 = 2;
            crossing_idx2 = 3;
            
        else % Still within this face.
            intersect_pt = ptstar;
            remainder = 0;
            crossing_idx1 = 0;
            crossing_idx2 = 0;
        end
    end

% Brute force find neighboring face with matching vertices. Rejects
% degenerate triangles.
    function [neighbor_face, neighbor_idx1, neighbor_idx2] = brute_force_vertices(face_num, idx1, idx2, faces, verts)
        vert1 = verts(faces(face_num,idx1),:);
        vert2 = verts(faces(face_num,idx2),:);
        
        vert_matches1 = find(max(abs(verts - repmat(vert1, [size(verts,1), 1])), [], 2) < 1e-8);
        vert_matches2 = find(max(abs(verts - repmat(vert2, [size(verts,1), 1])), [], 2) < 1e-8);
        
        for j = 1:size(faces,1)
            if (j == face_num)
                continue;
            end
            
            if (~isempty(intersect(faces(j,:), vert_matches1)) && ~isempty(intersect(faces(j,:), vert_matches2)))
                neighbor_face = j;
                
                neighbor_verts = verts(faces(neighbor_face,:),:);
                
                % No degenerate triangles please.
                if check_degeneracy(neighbor_verts)
                    continue;
                else
                    [~, neighbor_idx1, ~] = intersect(faces(j,:), vert_matches1);
                    [~, neighbor_idx2, ~] = intersect(faces(j,:), vert_matches2);
                    plot3(neighbor_verts(:,1), neighbor_verts(:,2), neighbor_verts(:,3),'.b', 'MarkerSize', 30)
                    return;
                end
            end
        end
        error('Could not match adjacent face with brute force.');
    end

    function degenerate = check_degeneracy(questionable_verts) % triangle vertices with each vertex on a row.
        %                         max(abs(cross(neighbor_verts(:,1), neighbor_verts(:,3)))) < 1e-6
        degenerate = max(abs(cross(questionable_verts(2,:) - questionable_verts(1,:), questionable_verts(3,:) - questionable_verts(1,:)))) < 1e-3 || ...
            max(abs(cross(questionable_verts(3,:) - questionable_verts(2,:), questionable_verts(1,:) - questionable_verts(2,:)))) < 1e-3;
    end
% Find the neighboring triangle to this one. Will try to be smart by
% using adjacency, but will resort to dumber means depending on how
% shitty the mesh is.
    function [neighbor_face, neighbor_idx1, neighbor_idx2] = locate_adjacent(face_num, idx1, idx2, faces, verts)
        [i1,j1] = ind2sub(size(faces),find(faces == faces(face_num,idx1))); % Find all matching first index.
        [i2,j2] = ind2sub(size(faces(i1,:)),find(faces(i1,:) == faces(face_num,idx2))); % Find matching second index in first set.
        
        face_matches = i1(i2);
        
        if (length(face_matches) == 1) % Weird case where the adjacency map isn't quite connected right, BUT we at least matched one vertex.
            disp('Cannot find a perfect adjacent match. Trying to see if any with at least 1 common node work.');
            % First vertex has matches. See if any of them are right.
            possible_match_indices = i1(i1 ~= face_num);
            possible_match_vertex_idx = j1(i1 ~= face_num);
            
            possible_matches = faces(possible_match_indices,:);
            to_match = repmat(verts(faces(face_num, idx2), :), [3,1]);
            for k = 1:length(possible_match_indices)
                [closest_dist, closest_idx] = min(sum(abs(verts(possible_matches(k,:),:) - to_match), 2));
                if closest_dist < 1e-4
                    neighbor_face = possible_match_indices(k);
                    neighbor_idx1 = possible_match_vertex_idx(k);
                    neighbor_idx2 = closest_idx;
                    if check_degeneracy(verts(faces(neighbor_face,:),:))
                        continue;
                    else
                        return;
                    end
                end
            end
            
            % Try it for the other index too.
            [i1,j1] = ind2sub(size(faces),find(faces == faces(face_num,idx2))); % Find all matching first index.
            possible_match_indices = i1(i1 ~= face_num);
            possible_match_vertex_idx = j1(i1 ~= face_num);
            
            possible_matches = faces(possible_match_indices,:);
            to_match = repmat(verts(faces(face_num, idx1), :), [3,1]);
            for k = 1:length(possible_match_indices)
                [closest_dist, closest_idx] = min(sum(abs(verts(possible_matches(k,:),:) - to_match), 2));
                if closest_dist < 1e-4
                    neighbor_face = possible_match_indices(k);
                    neighbor_idx1 = possible_match_vertex_idx(k);
                    neighbor_idx2 = closest_idx;
                    if check_degeneracy(verts(faces(neighbor_face,:),:))
                        continue;
                    else
                        return;
                    end
                end
            end
            disp('That did not work. Brute force matching vertices.');
            [neighbor_face, neighbor_idx1, neighbor_idx2] = brute_force_vertices(face_num, idx1, idx2, faces, verts);
            
        elseif (length(face_matches) == 2)
            face_match_logical = face_matches ~= face_num; % the OTHER matching face besides the original.
            
            neighbor_face = face_matches(face_match_logical);
            matching_first_idx = j1(i2);
            neighbor_idx1 = matching_first_idx(face_match_logical);
            neighbor_idx2 = j2(face_match_logical);
            
            if check_degeneracy(verts(faces(neighbor_face,:),:))
                disp('Regular face match was degenerate. Looking elsewhere');
                [neighbor_face, neighbor_idx1, neighbor_idx2] = brute_force_vertices(face_num, idx1, idx2, faces, verts);
            else
                return;
            end
            
        else % Likely will happen if there are NO matching vertices. Maybe do a brute force search?
            disp(length(face_matches));
            error('weird number of adjacency matches');
        end
    end
end