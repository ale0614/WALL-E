function movements= Inverse_Kinematics(wall_e, init_joints, transformations)

    points = size(transformations, 3);
    movements= zeros(points,6);
    previous_joints=init_joints(:);
    qlim = wall_e.qlim;
    
    for i=1: points
       current_pose=  transformations(:,:,i);
       new_joints = wall_e.ikine(current_pose, 'q0', previous_joints, 'mask', [1 1 1 1 1 1]);

       if isempty(new_joints)
        error("IK failed at point %d", i);
        end

       new_joints = new_joints(1, :);
       new_joints = previous_joints+ mod(new_joints - previous_joints + pi, 2*pi) - pi;

       if any(new_joints < qlim(:,1) - 1e-9) || any(new_joints > qlim(:,2) + 1e-9)
        error("Joint limit exceeded at point %d", i);
        end
              
       movements(i,:)=new_joints;
       previous_joints=new_joints;

    end  
end

