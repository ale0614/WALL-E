%% Main script: ties everything together (model code from Partner A goes where marked)
clear; clc; close all;
c = [0 0.55 0.35]; R = 0.15; m = [0 -1 0.35]; h = 0.07; N = 120;   % shared inputs
 
% --- Partner A: build the UR5e model here (the SerialLink code), call it wall_e ---
% wall_e = SerialLink(L, 'name', 'UR5e');  wall_e.base = transl(0,0,0);
 
[transformations, pts] = make_circle_poses(c, R, m, h, N);          % Partner A
init_joints = find_start_config(wall_e, transformations);           % Partner B
movements   = Inverse_Kinematics(wall_e, init_joints, transformations);
fprintf('Max joint step: %.1f deg\n', rad2deg(max(max(abs(diff(movements,1,1))))));
animate_wall_e(wall_e, movements, c, R, pts);