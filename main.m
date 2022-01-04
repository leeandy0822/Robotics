% Author : LEE-CHI-AN
% Date : 2022/1/3

clc
clear all
close all

% defind the orientation and position of A,B,C (cm)
A = [ 0  0  -1  40
     -1  0  0  -30
      0  1  0   10
      0  0  0   1 ];

B = [ 1  0  0  30
     0  -1  0  30
      0  0  -1  20
      0  0  0  1 ];

C = [ 0  1  0  40
      0 0  -1  20
      -1  0 0 -30
      0  0  0  1 ];

%% Choose which motion planning way 
number = input('1.Joint Motion Planning \n2.Cartesian Motion Planning\nPlease input (1 or 2):');

if number == 1
    % Choose the forth answer for IK 
    theta_A = [-36.8700   25.8000  -79.2270   53.4270 -126.8700 -180.0000];
    theta_B = [45.0000   12.2380  -88.9080  -13.3300  -90.0000  135.0000];
    theta_C = [26.5650   72.6480  -58.9930  -13.6550 -153.4350   90.0000];
    motion_planning_joint(theta_A,theta_B,theta_C,A,B,C);

elseif number == 2

    motion_planning_cartesian(A,B,C);

else
    fprintf('exit!\n');
end