%% WALL-E: UR5e draws a general circle on a sphere (Peter Corke Robotics Toolbox)
%  Run this file. Needs the Robotics Toolbox for MATLAB on the path.
clear; clc; close all;
addpath(fileparts(mfilename('fullpath')));      % find the functions next to this file

%% Shared inputs (change these to try other cases)
c = [0 0.55 0.35];      % sphere center (m)
R = 0.15;               % sphere radius (m)
m = [0 -1 0.35];        % normal of the circle's plane (tilted -> general case)
h = 0.07;               % plane offset from the sphere center, |h| < R
N = 120;                % points on the circle
base_pos = [0 0 0];     % robot base, outside the sphere

%% Robot (name, type, DH table and joint limits are documented in build_ur5e.m)
wall_e = build(base_pos);
wall_e                                           % prints the DH table

%% Target poses: circle on the sphere, tool z-axis along the surface normal
[transformations, pts] = make_circle(c, R, m, h, N);
fprintf('Base-to-sphere distance: %.3f m (UR5e reach 0.850 m)\n', norm(c - base_pos) - R);

%% Inverse kinematics (toolbox ikine), smooth and within joint limits
init_joints = start_config(wall_e, transformations);
movements   = Inverse_Kinematics(wall_e, init_joints, transformations);

%% Checks, plots, animation
results(wall_e, movements, transformations, c, R);
animate_wall_e(wall_e, movements, c, R, pts);          % add 'wall_e.mp4' to record a video
