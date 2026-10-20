function joint_solutions = ur5e_analytic_IK(wall_e, T)
% Calculate possible starting joint configurations for the first point on
% the circle using analytical inverse kinematics
%Later the results will be used to obtain the smoothest path


%Get the DH parameters from the robot model
L = wall_e.links;
offset = zeros(1,6); 
length = zeros(1,6); 
alpha = zeros(1,6);

%Store the dimensions of the six links
for j = 1:6
    offset(j) = L(j).d; 
    length(j) = L(j).a; 
    alpha(j) = L(j).alpha;
end

%Get the target pose relative to the robot base
T = double(wall_e.base) \ T;

%Extract the orientation vectors and end-Effector position
x_axis = T(1:3,1); 
y_axis = T(1:3,2); 
z_axis = T(1:3,3); 
position = T(1:3,4);

%Calculate the wrist reference point
wrist_pos = position - offset(6)*z_axis;
rho = hypot(wrist_pos(1), wrist_pos(2));
joint_solutions = zeros(0,6);

% Check if the wrist position is too close to the base
if rho < abs(offset(4)), return; end           

%Calculate the possible angle needed to determine joint 1
base_angle = atan2(wrist_pos(2), wrist_pos(1));  
angle_offset = asin(offset(4)/rho);

%Try two possible shoulder configurations
for q1 = [base_angle + angle_offset, base_angle + pi - angle_offset] 
    %Calculate the direction vector on joint 1
    z1 = [sin(q1); -cos(q1); 0];
    %Calculate the cosine value for joint 5
    c5 = (position.'*z1 - offset(4)) / offset(6);
    
    %Skip it if invalid
    if abs(c5) > 1 + 1e-12,continue; end

    %Calculate 2 possible configurations for joint 5
    for q5 = [acos(max(-1,min(1,c5))), -acos(max(-1,min(1,c5)))]  
        s5 = sin(q5);
        %Skip configurations to avoid singularity
        if abs(s5) < 1e-9, continue; end 
        
        % Calculate the angle for joint 6
        q6 = atan2(-(z1.'*y_axis)/s5, (z1.'*x_axis)/s5);
        
        %Find transformations of the remaining three joints
        M = dh_A(q1,offset(1), ...
            length(1),alpha(1)) \ T / dh_A(q6,offset(6), ...
            length(6),alpha(6)) / dh_A(q5,offset(5), ...
            length(5),alpha(5));

        %Get the combined angle and position of the middle joints
        th234 = atan2(M(2,1), M(1,1));  
        px = M(1,4);  py = M(2,4);
        c3 = (px^2 + py^2 - length(2)^2 - length(3)^2) / (2*length(2)*length(3));
        %Check for a valid cosine value
        if abs(c3) > 1 + 1e-12,continue;end

         
        % Try two possible elbow configurations
        for q3 = [acos(max(-1,min(1,c3))), -acos(max(-1,min(1,c3)))]  
            % Calculate joint 2 using the robot geometry
            q2 = atan2(py, px) - atan2(length(3)*sin(q3), length(2) + length(3)*cos(q3));
            
            % Calculate the remaining angle for joint 4
            q4 = th234 - q2 - q3;
            % Store the six joint angles
            joint_solutions(end+1,:) = [q1 q2 q3 q4 q5 q6]; 
        end
    end
end

%Keep the joints angles in the range
joint_solutions = mod(joint_solutions + pi, 2*pi) - pi;
end

%Calculate DH transformation matrix
function A = dh_A(theta, offset, length, alpha)
ct = cos(theta); st = sin(theta); ca = cos(alpha); sa = sin(alpha);

A = [ct, -st*ca,  st*sa, length*ct;
    st,  ct*ca, -ct*sa, length*st;
    0,   sa,     ca,    offset;
    0,   0,      0,     1];
end
