%% Global Variables

clear;
close all;

%% Variables Definitions

simul_CalibScript_KR30;
kr30 = readtable('data_kr30_dkp400');
n_meas_points = size(kr30.x, 1);
%n_meas_points = 10;

disp(T_bt);

%%
P_b_meas_kr30=[];

for i=1:n_meas_points
    aux = [kr30.x(i) kr30.y(i) kr30.z(i)]';
    P_b_meas_kr30 = [P_b_meas_kr30 aux];
    
    %Adding the wire's size for each point
%     R_be=rpy2r(kr30.a(i), kr30.b(i), kr30.c(i));
%     aux = [kr30.x(i) kr30.y(i) kr30.z(i)]';
%     new_aux = R_be(:,3)*tcp + aux;
%     P_b_meas_kr30 = [P_b_meas_kr30 new_aux];
end

%%

T_b_t_hat = transl([1.4 0.1 -0.5])*trotz(180);

% Experimental Data: Position with respect to KR30 base (KR30)
% p0_kr30 = [-0.632 -0.753 0.351 1]';
% p1_kr30 = [-0.628 -1.943 0.368 1]';
% p2_kr30 = [0.137 -1.583 0.654 1]';

% P_b_meas_kr30 = [p0_kr30 p1_kr30 p2_kr30];

disp(T_b_t_hat);

%% Estimated values for pose (Don't change)

x_bt_hat_p = T_b_t_hat(1:3,4);
x_bt_hat_R = rotm2eul(T_b_t_hat(1:3,1:3),'zyx');

x_bt_hat = [x_bt_hat_p' x_bt_hat_R];

% Vectorial notation for HT calibrated by KUKA Technician (KUKA TP)

x_bt_p = T_bt(1:3,4);
x_bt_R = rotm2eul(T_bt(1:3,1:3),'zyx');

x_bt = [x_bt_p' x_bt_R];

%% Script

n_iter = 2000;
beta = 0.5; % gain

N = [];     % norm value in each iteration
F = [];     % objective function in each iteration
X = [];     % homogenous transformation in vectorial notation

% Selecting the calibration points based on the number of points chosen
P_t = P_t(:,1:n_meas_points);
P_b = P_b(:,1:n_meas_points);

for i=1:n_iter
    f = obj_fun(x_bt_hat, P_b_meas_kr30, P_t);
    
	jacob_f = jacobian_fun(x_bt_hat, P_t);
    
    pinv_jacob = pinv(jacob_f);
    
    new_x_bt_hat = x_bt_hat - (beta * pinv_jacob * f)';
    
    dXbt = new_x_bt_hat - x_bt_hat;
    x_bt_hat = new_x_bt_hat;
    
    dPos = norm(dXbt(1:3));
    dPhi = abs(dXbt(4));
    dTheta = abs(dXbt(5));
    dPsi = abs(dXbt(6));
    
    eps_pos = 0.00001/1000;
    eps_ori = deg2rad(0.00001);
    
    norm_value = norm(f);
    F = [F f];
    N = [N norm_value];
    X = [X; x_bt_hat];
    
    stop_iter = i;
        
    if (dPos < eps_pos && dPhi < eps_ori && dTheta < eps_ori && dPsi < eps_ori)
        fprintf('Stop condition on iteration %d \n', i);
        break;
    end
end

I = 1:stop_iter;

% Vectorial notation of calibrated HT after the Newton's Method

T_b_t_hat_final = [eul2rotm(x_bt_hat(4:6),'zyx') x_bt_hat(1:3)'; 0 0 0 1];

x_bt_hat_final_p = T_b_t_hat_final(1:3,4);
x_bt_hat_final_R = rotm2eul(T_b_t_hat_final(1:3,1:3),'zyx');

x_bt_hat_final = [x_bt_hat_final_p' x_bt_hat_final_R];

disp(T_b_t_hat_final);

% Norm's difference between the initial estimative and the result after the
% Newton's Method (in milimeter)

norm_diff = (norm(x_bt_p) - norm(x_bt_hat_final_p))*1000;

%%
% Position error between initial estimation and final estimation by Newton's Method (in mm)

% T_bt is an initial estimative based on URDF
diff_Tbt = (T_bt(1:3,4)-T_b_t_hat_final(1:3,4))*1000;

% Here I used the calibrated value (via Teach Pendant)
% calib_value = [0.89454 0.008 -0.48292];
% calib_value = [1.3925 0.1005 -0.5034]';
% diff_Tbt = (calib_value' - T_b_t_hat_final(1:3,4))*100;

% Difference between the points measurement in mm (experiment vs matlab)

diff_Pb = (P_b_meas_kr30 - P_b)*1000;     

% Norm of this difference for each calibration point
norm_diff_points = [];
for i=1:size(diff_Pb,2)
   norm_diff_points = [norm_diff_points; norm(diff_Pb(:,i))]; 
end

%%
% Position error between initial estimation and final estimation by Newton's Method (in mm)

diff_Tbt = (T_bt(1:3,4)-T_b_t_hat_final(1:3,4))*1000;

% Difference between the points measurement in cm (experiment vs matlab)

diff_Pb = (P_b_meas_kr30-P_b)*1000;

function y = obj_fun(x_bt, P, P_t)

    n_points = length(P);
    
    p_bt = x_bt(1:3);
    R_bt = eul2rotm(x_bt(4:6),'zyx');
    T_bt = [R_bt p_bt(:); 0 0 0 1];

    y_matrix = P(1:3,:) - T_bt(1:3,:)*P_t;
    y = y_matrix(:);
end


%% Jacobian Matrix

function y = jacobian_fun(x_bt, P)
    [~,n_points] = size(P);
    
    phi = x_bt(4); % z
    theta = x_bt(5); % y
    psi = x_bt(6); % x
    
    % Jacobian matrix for position and orientation
    df_dpbt = [];
    df_drbt = [];
    
    % Derivatives
    dr_dphi = [-sin(phi) -cos(phi) 0; cos(phi) -sin(phi) 0; 0 0 0]*roty(theta)*rotx(psi);   % z
    dr_dtheta = rotz(phi)*[-sin(theta) 0 cos(theta); 0 0 0; -cos(theta) 0 -sin(theta)]*rotx(psi);   % y
    dr_dpsi = rotz(phi)*roty(theta)*[0 0 0; 0 -sin(psi) -cos(psi); 0 cos(psi) -sin(psi)];   % x
    
    % this loop calculates each "block line" on jacobian matrix
    for i=1:n_points
        p_i = P(1:3,i);
        part_deriv = [dr_dphi*p_i dr_dtheta*p_i dr_dpsi*p_i];
        df_drbt = [df_drbt; -part_deriv];
        df_dpbt = [df_dpbt; -eye(3)];
    end
    
    jacob_f = [df_dpbt df_drbt];
    y = jacob_f;

end