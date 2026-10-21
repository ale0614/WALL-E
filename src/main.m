clear; clc; close all;
addpath(fileparts(mfilename('fullpath')));
% This is the main file of our project. It puts everything together,
% allows us to define the initial parameters, and passes that information
% to the other functions.

%Define the circle and sphere parameters
c = [0 0.55 0.35];     
R = 0.15;               
m = [0 -1 0.35];        
h = 0.07;               
N = 120; 
%Robot's base outside the sphere
base_pos = [0 0 0];     

%Build and display the robot
wall_e = build(base_pos);
wall_e                                         

%Generate the circle trajectory
[transformations, pts] = make_circle(c, R, m, h, N);
fprintf('Base-to-sphere distance: %.3f m (UR5e reach 0.850 m)\n', norm(c - base_pos) - R);

%Calculate the inverse kinematics and find the angles and movements 
init_joints = start_config(wall_e, transformations);
movements   = Inverse_Kinematics(wall_e, init_joints, transformations);

%Show results and the animated robot
results(wall_e, movements, transformations, c, R);
animate_wall_e(wall_e, movements, c, R, pts);
