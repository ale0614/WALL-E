function movements = Inverse_Kinematics(wall_e, init_joints, transformations)
% INVERSE_KINEMATICS  Joint angles for every pose in transformations (4x4xN).
%   Each solve is seeded with the previous solution (toolbox ikine), and each joint is
%   shifted by a multiple of 2*pi to be closest to the previous one, so no jumps.
%   Errors out if IK did not truly reach the pose or a joint limit is exceeded.
points = size(transformations, 3);
movements = zeros(points, 6);
previous_joints = init_joints(:)';          % ROW vector (1x6), like ikine returns
qlim = wall_e.qlim;                         % 6x2 joint limits

for i = 1:points
    current_pose = transformations(:,:,i);
    new_joints = wall_e.ikine(current_pose, 'q0', previous_joints, 'mask', [1 1 1 1 1 1]);
    if isempty(new_joints)
        error('IK failed at point %d', i);
    end
    new_joints = new_joints(1,:);
    % some toolbox versions return the last iterate when not converged: verify
    if norm(DH_FK(wall_e, new_joints) - current_pose, 'fro') > 5e-3
        error('IK did not converge at point %d', i);
    end
    % unwrap: choose the 2*pi-equivalent angle closest to the previous angle
    new_joints = previous_joints + mod(new_joints - previous_joints + pi, 2*pi) - pi;
    if any(new_joints < qlim(:,1)' - 1e-9) || any(new_joints > qlim(:,2)' + 1e-9)
        error('Joint limit exceeded at point %d', i);
    end
    movements(i,:) = new_joints;
    previous_joints = new_joints;
end
end
