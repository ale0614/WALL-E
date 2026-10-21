function movements = Inverse_Kinematics(wall_e, init_joints, transformations)
%This function calculate the joint angles for each point of the circular
%path using numerical kinematics 

%Get number of points
points = size(transformations, 3);
%Create an array to store the six joint angles
movements = zeros(points, 6);

%Set the Initial joint angles and get their limits
previous_joints = init_joints(:)';          
qlim = wall_e.qlim;                         

%Loop to calculate the joint angles for each point
for i = 1:points
    %Get target pose of the current position
    current_pose = transformations(:,:,i);

    %Peter Corke's numerical IK formula to calculate the joint angles
    new_joints = wall_e.ikine(current_pose, 'q0', previous_joints, 'mask', [1 1 1 1 1 1]);
    
    %Check if we found solution
    if isempty(new_joints)
        error('IK failed at point %d', i);
    end

    %Get the joint angles
    new_joints = new_joints(1,:);

    % Check to see if calculated joint angles reach the target pose
    if norm(DH_FK(wall_e, new_joints) - current_pose, 'fro') > 5e-3
        error('IK did not converge at point %d', i);
    end
    
    %Choose the closest equivalent joints
    new_joints = previous_joints + mod(new_joints - previous_joints + pi, 2*pi) - pi;
    
    %Check joints are inside the limits
    if any(new_joints < qlim(:,1)' - 1e-9) || any(new_joints > qlim(:,2)' + 1e-9)
        error('Joint limit exceeded at point %d', i);
    end
    %Update the joint angles for current point
    movements(i,:) = new_joints;

    %Update the joint angles for next point
    previous_joints = new_joints;
end
end
