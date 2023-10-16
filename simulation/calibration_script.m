clear all;

global kp2;
global dkp400;

%% RPY Angles (radian)
% x = psi; (Roll)
% y = theta; (Pitch)
% z = phi; (Yaw)

%% Serial Link Definitions

kp2 = KP2_URDF_SerialLink;
dkp400 = DKP400_URDF_SerialLink;

%%
n_points = 1000;
%T_t_d = kp2.fkine([pi/4 pi/2]); % KP2
T_t_d = dkp400.fkine([pi/4 0]);  % DKP400
%T_t_d = transl([0 0 1]);        % Translação de 1 unidade em z

P_t = create_points(T_t_d.T,n_points);

%%
% Initial simulation
% angles = deg2rad([45 12 15]);
% T_b_t = transl([1 1 0])*eul2tform(angles,'zyx');

% KUKA DKP 400
% angles = deg2rad([180 0 0]);
angles = [3.14 0.0 0.0];
T_b_t = transl([1.39148 0.10049 -0.503])*eul2tform(angles,'zyx');

% KUKA KP2 HV1100HW
% angles = [-3.1361 0.0007 0.0002];
% T_b_t = transl([1.7418 0.3981 -0.4659])*eul2tform(angles,'zyx');

[P_b] = points(P_t,T_b_t);

%%

function P_t = create_points(T,n_points)

    I = ones(1,n_points);
    P_d_xy = 1*rand(2,n_points) - 0.5;
    P_d_z = 0*I;
    
    P_t = T(1:3,:)*[P_d_xy; P_d_z; I];
end


function [P] = points(Pin,T)

	[~,n_points] = size(Pin);
    ones_aux = ones(1,n_points);

    P = T(1:3,:)*[Pin;ones_aux];

end

