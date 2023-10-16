%% RPY Angles (radian)
% clear
global kp2

% x = psi; (Roll)
% y = theta; (Pitch)
% z = phi; (Yaw)

%% Definitions

kp2 = KP2_URDF_SerialLink;
kr90 = readtable('data_kr90_kp2.xlsx');
n_meas_points = size(kr90.x, 1);

% Excel table with informations about each robot
% kr90 = readtable('tabela_dados_calibracao.xlsx', 'Sheet', 'kr90_new');

%%

% KR90 - Position of calibration points with respect to the Depos. Frame
% p0 = [-0.595 -0.660 0.846 1]';
% p1 = [0.595 -0.660 0.846 1]';

% These points don't depend on the kp2 angles (Ref. to the positioner frame
p0 = [-0.600 -0.790 0.846 1]';
p1 = [0.600 -0.790 0.846 1]';
% This point don't depend on the kp2 e2 joint
p2 = [-0.279 -0.395 -0.146 1]';
p3 = [0 0.4 -0.014 1]';
p4 = [0.35 0 0.051 1]';
p5 = [-0.35 0 0.051 1]';

P_d = [p0 p1 p2 p3 p4 p5];

%%
n_points = size(P_d, 2);
T_t_d = kp2.fkine([0 0]);

P_t = create_points(P_d);

%%
% Alterar esses valores de Tbt para o resultado obtido após a aplicação
% do algoritmo; uma possível métrica é comparar os valores de P_b
% calculados no MATLAB com a estimativa original e ver se teve alguma
% melhora.

% Initial estimative
% angles = deg2rad([180 0 0]);
% T_b_t = transl([1.7392 0.3920 -0.4636])*eul2tform(angles,'zyx');

% Calibration after Newton's Method (+1 cm em y: correção após testes)
angles = [-3.1361 0.0007 0.0002];
T_b_t = transl([1.7418 0.3981 -0.4659])*eul2tform(angles,'zyx');

[P_b] = points(P_t,T_b_t);
[~,n_points] = size(P_b);

%%

% Position of points related to table frame
function y = create_points(P_d)
    global kp2;
    
    pt0 = P_d(:,1);
    pt1 = P_d(:,2);
    % Point 2
    pt2 = (kp2.fkine([pi/2 0]).T)*P_d(:,3);
    pt3 = (kp2.fkine([pi/4 0]).T)*P_d(:,3);
    % Point 3 (Rever esses pontos; erro considerável em y)
    pt4 = (kp2.fkine([pi/2 -pi]).T)*P_d(:,4);
    pt5 = (kp2.fkine([pi/2 -(5/6)*pi]).T)*P_d(:,4);
    pt6 = (kp2.fkine([pi/2 (5/6)*pi]).T)*P_d(:,4);
    pt7 = (kp2.fkine([pi/4 (5/6)*pi]).T)*P_d(:,4);
    pt8 = (kp2.fkine([pi/4 pi]).T)*P_d(:,4);
    %pt9 = (kp2.fkine([pi/4 -(5/6)*pi]).T)*P_d(:,4);
    % Point 4
    pt10 = (kp2.fkine([pi/4 (5/6)*pi]).T)*P_d(:,5);
    pt11 = (kp2.fkine([pi/4 pi]).T)*P_d(:,5);
    pt12 = (kp2.fkine([pi/4 -(5/6)*pi]).T)*P_d(:,5);
    pt13 = (kp2.fkine([0 -pi]).T)*P_d(:,5);
    pt14 = (kp2.fkine([0 pi/2]).T)*P_d(:,5);
    % Point 5
    pt15 = (kp2.fkine([pi/4 (5/6)*pi]).T)*P_d(:,6);
    pt16 = (kp2.fkine([pi/4 pi]).T)*P_d(:,6);
    pt17 = (kp2.fkine([pi/4 -(5/6)*pi]).T)*P_d(:,6);
    pt18 = (kp2.fkine([0 -pi]).T)*P_d(:,6);
    pt19 = (kp2.fkine([0 pi/2]).T)*P_d(:,6);
    
%     pt0 = P_d(:,1);
%     pt1 = P_d(:,2);
%     % Point 2
%     pt2 = (kp2.fkine([pi/2 0]).T)*P_d(:,3);
%     pt3 = (kp2.fkine([pi/4 0]).T)*P_d(:,3);
%     % Point 3
%     pt4 = (kp2.fkine([pi/2 -pi]).T)*P_d(:,4);
%     pt6 = (kp2.fkine([pi/2 (5/6)*pi]).T)*P_d(:,4);
%     pt7 = (kp2.fkine([pi/4 (5/6)*pi]).T)*P_d(:,4);
%     pt8 = (kp2.fkine([pi/4 pi]).T)*P_d(:,4);
%     % Point 4
%     pt10 = (kp2.fkine([pi/4 (5/6)*pi]).T)*P_d(:,5);
%     pt11 = (kp2.fkine([pi/4 pi]).T)*P_d(:,5);
%     pt13 = (kp2.fkine([0 -pi]).T)*P_d(:,5);
%     pt14 = (kp2.fkine([0 pi/2]).T)*P_d(:,5);
%     % Point 5
%     pt15 = (kp2.fkine([pi/4 (5/6)*pi]).T)*P_d(:,6);
%     pt18 = (kp2.fkine([0 -pi]).T)*P_d(:,6);
%     pt19 = (kp2.fkine([0 pi/2]).T)*P_d(:,6);

%     P_t = [];
%     m_table_configs = size(table_joints,1);
%     for i = 1:m_table_configs
%         thetai = table_joints(i,:);
%         T_t_d = kp2.fkine(thetai);
%         T_matrix = T_t_d.T;
%         aux = T_matrix(:,:)*P_d;
%         P_t = [P_t aux]; 
%     end
    
    P_t = [pt0 pt1 pt2 pt3 pt4 pt5 pt6 pt7 pt8 pt10 pt11 pt12 pt13 pt14 pt15 pt16 pt17 pt18 pt19];
    %P_t = [pt0 pt1 pt2 pt3 pt4 pt6 pt7 pt8 pt10 pt11 pt13 pt14 pt15 pt18 pt19];

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