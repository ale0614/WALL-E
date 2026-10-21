function T = DH_FK(wall_e, q)
% The function calculate the forward kinematics using the robot's DH table parameters 

%Get the links 
L = wall_e.links;
T = double(wall_e.base);

%Calculate the transformations of the six joints
for j = 1:6
    %Calculate the sin and cos
    th = q(j) + L(j).offset;
    ct = cos(th);
    ca = cos(L(j).alpha);
    st = sin(th); 
    sa = sin(L(j).alpha);

    %Calculate the DH transformation matrix for each joint
    T = T * [ct, -st*ca,  st*sa, L(j).a*ct;
        st,  ct*ca, -ct*sa, L(j).a*st;
        0,   sa,     ca,    L(j).d;
        0,   0,      0,     1];
end
%Obtain the final position and orientation of the robot tool to its base
T = T * double(wall_e.tool);
end
