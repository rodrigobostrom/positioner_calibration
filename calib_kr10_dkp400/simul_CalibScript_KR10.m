%% RPY Angles (radian)
clear
global dkp400

% x = psi; (Roll)
% y = theta; (Pitch)
% z = phi; (Yaw)

%% Definitions

dkp400 = DKP400_URDF_SerialLink;

% Excel table with informations about each robot
kr10 = readtable('data_kr10_dkp400');

%%

% KR10 - Position of calibration points with respect to the Depos. Frame
% (tool0 - center of the table)

p0 = [0 -0.21 0 1]';
p1 = [-0.1 0 0 1]';
p2 = [-0.0707 -0.0707 0 1]';
p3 = [-0.0707 0.0707 0 1]';
p4 = [-0.23 0 0 1]';
p5 = [-0.234 0 -0.01 1]';
p6 = [0.6 0 -0.347 1]';

P_d = [p0 p1 p2 p3 p4 p5 p6];

%%
n_points = size(P_d, 2);
T_t_d = dkp400.fkine([0 0]);

P_t = create_points(P_d);

%%

% P_b: points' coordinates 

% Calibrated value (URDF)
% xyz="1.73923 0.39198 -0.4636" rpy="0 0 3.1416"

% KR10 - Calibrated value - KUKA technician - KUKA Teach Pendant Method
angles = deg2rad([-90.29 -0.02 0.4]);
T_b_t = transl([0.89454 0.008 -0.48292])*eul2tform(angles,'zyx');

% New estimative based on results of calibration
% angles = deg2rad([-0.0588 -0.0136 -0.0324]);
% T_b_t = transl([0.9402 0.0112 -0.4706])*eul2tform(angles,'zyx');

[P_b] = points(P_t,T_b_t);
[~,n_points] = size(P_b);

%%

% Position of points related to table frame
function y = create_points(P_d)
    global dkp400;
    
    table_joints = [0 0;
        0 pi/2;
        0 -pi/2;
       -pi/4 0;
       -pi/4 pi/2;
       -pi/6 0];  
    
    % table_joints(i,:) is the joint configuration based on definition above 
    % each index corresponds to the ones on excel table
    
    % Point 0
    pt0 = (dkp400.fkine(table_joints(1,:)).T)*P_d(:,1);
    pt1 = (dkp400.fkine(table_joints(2,:)).T)*P_d(:,1);
    pt2 = (dkp400.fkine(table_joints(4,:)).T)*P_d(:,1);
    pt3 = (dkp400.fkine(table_joints(5,:)).T)*P_d(:,1);
    
    % Point 1
    
    pt4 = (dkp400.fkine(table_joints(1,:)).T)*P_d(:,2);
    pt5 = (dkp400.fkine(table_joints(3,:)).T)*P_d(:,2);
    pt6 = (dkp400.fkine(table_joints(2,:)).T)*P_d(:,2);
    
    % Point 2
    
    pt7 = (dkp400.fkine(table_joints(1,:)).T)*P_d(:,3);
    pt8 = (dkp400.fkine(table_joints(3,:)).T)*P_d(:,3);
    pt9 = (dkp400.fkine(table_joints(6,:)).T)*P_d(:,3);
    
    % Point 3
    
    pt10 = (dkp400.fkine(table_joints(1,:)).T)*P_d(:,4);
    pt11 = (dkp400.fkine(table_joints(3,:)).T)*P_d(:,4);
    
    % Point 4
    
    pt12 = (dkp400.fkine(table_joints(1,:)).T)*P_d(:,6);
    pt13 = (dkp400.fkine(table_joints(2,:)).T)*P_d(:,5);
    
    % Point 5
    
    pt14 = (dkp400.fkine(table_joints(1,:)).T)*P_d(:,7);
    
    P_t = [pt0 pt1 pt2 pt3 pt4 pt5 pt6 pt7 pt8 pt9 pt10 pt11 pt12 pt13 pt14];
    %P_t = T(1:3,:)*P_d;
    
    y = P_t;
end

% Position of points related to the robot frame
function [P] = points(Pin,T)

    % Pin = P_t
	[~,n_points] = size(Pin);
    ones_aux = ones(1,n_points);

    P = T(1:3,:)*Pin;

end