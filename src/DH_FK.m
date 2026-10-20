function T = DH_FK(wall_e, q)
% DH_FK  Plain-matrix forward kinematics (4x4) from the robot's own DH data.
%   Works in any toolbox version (fkine returns SE3 objects in newer ones) and is
%   fast. Used to verify IK results and to read the end-effector pose.
L = wall_e.links;
T = double(wall_e.base);
for j = 1:6
    th = q(j) + L(j).offset;
    ct = cos(th); st = sin(th); ca = cos(L(j).alpha); sa = sin(L(j).alpha);
    T = T * [ct, -st*ca,  st*sa, L(j).a*ct;
        st,  ct*ca, -ct*sa, L(j).a*st;
        0,   sa,     ca,    L(j).d;
        0,   0,      0,     1];
end
T = T * double(wall_e.tool);
end
