function quatspan = quatintegrate(tspan, quat_init, omegaspan)
% Still feels like a dumb thing to do, especially since we have a full
% expression for omega. Still, it's better than euler from before. If the
% normalization is taken away from both rk4 and euler, euler loses unit
% length by about a factor of bajillion more.

validateattributes(tspan, {'numeric'}, {'real', 'increasing', 'vector'});
validateattributes(quat_init, {'numeric'}, {'real', 'vector', '>=', -1, '<=', 1});
validateattributes(omegaspan, {'numeric'}, {'real', 'ncols', 3, 'nrows', length(tspan)});

if iscolumn(quat_init)
    quat_init = quat_init';
end

quatspan = zeros(length(tspan), 4);
quatspan(1,:) = quat_init;

curr_quat = quat_init;
for i = 1:length(tspan) - 1
    h = tspan(i + 1) - tspan(i);
    
    %     k_1 = F_xy(x(i),y(i));
    k_1 = qdot(curr_quat, omegaspan(i,:));
    
    %     k_2 = F_xy(x(i)+0.5*h,y(i)+0.5*h*k_1);
    k_2 = qdot(curr_quat + 0.5*h*k_1, (omegaspan(i,:) + omegaspan(i + 1,:))/2);
    
    %     k_3 = F_xy((x(i)+0.5*h),(y(i)+0.5*h*k_2));
    k_3 = qdot(curr_quat + 0.5*h*k_2, (omegaspan(i,:) + omegaspan(i + 1,:))/2);
    
    %     k_4 = F_xy((x(i)+h),(y(i)+k_3*h));
    k_4 = qdot(curr_quat + h*k_3, omegaspan(i + 1,:));
    
    curr_quat = quatspan(i, :) + (1/6)*(k_1+2*k_2+2*k_3+k_4)*h;
    curr_quat = curr_quat/norm(curr_quat); % Keeps unit length well anyway, but never hurts if time is not a big issue.
    quatspan(i + 1, :) = curr_quat;
end
quatspan(:,1) = -quatspan(:,1); % Matlab has some left-handed stuff thrown around.

validateattributes(quatspan, {'numeric'}, {'real', 'ncols', 4, 'nrows', length(tspan)});

    function qdt = qdot(current_q, current_omega)
        qdt = 0.5 * quatmultiply([0, current_omega], current_q);
    end
end

