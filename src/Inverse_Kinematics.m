function movements= Inverse_Kinematics(wall_e, init_joints, transformations)

    points = size(transformations, 3);
    movements= zeros(points,6);
    previous_joints=init_joints;
    
    for i=1: points
       current_pose=  transformations(:,:,i);
       new_joints = wall_e.ikine(current_pose, previous_joints);
       
       movements(i,:)=new_joints;
       previous_joints=new_joints;

    end 
    
end

