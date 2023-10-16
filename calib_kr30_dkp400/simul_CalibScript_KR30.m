%% RPY Angles (radian)

global dkp400

% x = psi; (Roll)
% y = theta; (Pitch)
% z = phi; (Yaw)

%% Definitions

dkp400 = DKP400_URDF_SerialLink;
kr30 = readtable('data_kr30_dkp400');
n_meas_points = size(kr30.x, 1);

%%

% KR30 - Position of calibration points with respect to the Depos. Frame
% Same points used on the KR90 + KP2 Calibration - Same index
% Except for the point 7

% KUKA KP2 HV1100

% First two points don't dependent on the kp2 angles
% p0_kr30 = [-0.600 -0.790 0.846 1]';
% p1_kr30 = [0.600 -0.790 0.846 1]';
% p2_kr30 = [-0.280 -0.400 -0.100 1]';
% p4_kr30 = [0.4 0 0.049 1]';
% % Only point different from the ones defined before
% p7_kr30 = [0 0.24 0.049 1]';
% P_d_kr30 = [p0_kr30 p1_kr30 p7_kr30 p2_kr30 p4_kr30];

% KUKA DKP400

% p0_kr30 = [0 0.21 0 1]';
p0_kr30 = [0.21 0 0 1]';

P_d_kr30 = [p0_kr30];

%% Definição dos pontos de calibração com relação ao sist. de coord. da mesa

T_t_d = dkp400.fkine([0 0]);
P_t = create_points(P_d_kr30, n_meas_points, kr30);

%%

% Definição da posição dos pontos de calibração em relação a base do robô
% Usa-se a posição calculada acima e aplica-se a transf. homôgenea estimada
% da base do robô para encontrar esses valores

% Real value
% Rot_bt = rotz(-1.5681)*roty(-0.0138)*rotx(0.0017);

% KUKA KP2 HV1100
% Rot_bt = rotz(-89.84)*roty(-0.79)*rotx(0.1);
% P_bt = [0.0414 -1.3574 -0.4841]';
% T_bt = [Rot_bt P_bt; 0 0 0 1];

% KUKA DKP400
Rot_bt = rotz(180.10)*roty(-0.2)*rotx(0.21);
P_bt = [1.3925 0.1005 -0.5034]';
T_bt = [Rot_bt P_bt; 0 0 0 1];

% yaw = rad2deg(3.14);
% pitch = rad2deg(-0.0004);
% roll = rad2deg(0.0010);
% Rot_bt = rotz(yaw)*roty(pitch)*rotx(roll);
% P_bt = [1.5178 0.0070 -0.4976]';
% T_bt = [Rot_bt P_bt; 0 0 0 1];

[P_b] = points(P_t,T_bt);
[~,n_points] = size(P_b);

%%

% Position of points related to table frame
% The index of Pt refers to the index in the excel table of each point
function y = create_points(P_d, n_meas_points, kr30)
    
    global dkp400;

    P_t = [];

    for i=1:n_meas_points
        point_i = (dkp400.fkine([kr30.e1(i) kr30.e2(i)]).T)*P_d(:,1);
        P_t = [P_t point_i];
    end
    
    y = P_t;
         
%     pt0 = P_d(:,1);
%     pt1 = P_d(:,2);
%     pt2 = (kp2.fkine([0 -pi/2]).T)*P_d(:,3);
%     pt3 = (kp2.fkine([pi/4 0]).T)*P_d(:,4);
%     pt4 = (kp2.fkine([pi/4 0]).T)*P_d(:,5);
%     pt5 = (kp2.fkine([0 0]).T)*P_d(:,5);
%     
%     P_t = [pt0 pt1 pt2 pt3 pt4 pt5];
%     y = P_t;

end

% Position of points related to the robot frame
function [P] = points(Pin,T)

    % Pin = P_t
	[~,n_points] = size(Pin);
    ones_aux = ones(1,n_points);

    P = T(1:3,:)*Pin;

end

% Experimental points: position related to base robot frame
% function P_base = setCalibPoints(T, calib_data)
%     tab_points = [];
%     vec = [];
%     n_points = length(calib_data.x);
%     for i=1:n_points
%         vec = [calib_data.x(i) calib_data.y(i) calib_data.z(i)]'; 
%         tab_points = [tab_points vec];
%     end
%     P_base = tab_points;
% end