%% Global Variables

close all;

%% Variables Definitions
% Maximum number of points: 15

simul_CalibScript_KR10;
kr10 = readtable('data_kr10_dkp400');
n_meas_points = size(kr10.x, 1);
% n_meas_points = 15;

disp(T_b_t);

%%
% Experimental Data (KR10)
% HT: Homogeneous Transformation

P_b_meas_kr10=[];
tcp = 0.015;  % arame: 15 mm

for i=1:n_meas_points
    
    % Measurement without the wire
%     aux = [kr10.x(i) kr10.y(i) kr10.z(i)]';
%     P_b_meas_kr10 = [P_b_meas_kr10 aux];

    % Adding the wire's size for each point
    R_be=rpy2r(kr10.a(i), kr10.b(i), kr10.c(i));
    aux = [kr10.x(i) kr10.y(i) kr10.z(i)]';
    new_aux = aux - R_be(:,3)*tcp;
    P_b_meas_kr10 = [P_b_meas_kr10 new_aux];
end

angles = deg2rad([-90 0 0]);
T_b_t_hat = transl([1.0 0.01 -0.46])*eul2tform(angles,'zyx');

disp(T_b_t_hat);

%% Estimated values for pose (Don't change)

% Vectorial notation for initial estimative of homogeneous transformation

x_bt_hat_p = T_b_t_hat(1:3,4);
x_bt_hat_R = rotm2eul(T_b_t_hat(1:3,1:3),'zyx');

x_bt_hat = [x_bt_hat_p' x_bt_hat_R];

% Vectorial notation for HT calibrated  by KUKA Technician (KUKA TP)

x_bt_p = T_b_t(1:3,4);
x_bt_R = rotm2eul(T_b_t(1:3,1:3),'zyx');

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
    f = obj_fun(x_bt_hat, P_b_meas_kr10, P_t);
    
	jacob_f = jacobian_fun(x_bt_hat, P_t);
    
    pinv_jacob = pinv(jacob_f);
    
    new_x_bt_hat = x_bt_hat - (beta * pinv_jacob * f)';
    
    dXbt = new_x_bt_hat - x_bt_hat;
    
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
    
    % State update
    x_bt_hat = new_x_bt_hat;
    
    stop_iter = i;
        
    if (dPos < eps_pos && dPhi < eps_ori && dTheta < eps_ori && dPsi < eps_ori)
        fprintf('Stop condition on iteration %d \n', i);
        break;
    end
end

% fprintf('The norm of function in this iteration is %.8f \n \n', norm_value);
% fprintf('Vectorial notation of the last iteration (X_BT):\n');
% x_bt

I = 1:stop_iter;

% Vectorial notation of calibrated HT after the Newton's Method

T_b_t_hat_final = [eul2rotm(x_bt_hat(4:6),'zyx') x_bt_hat(1:3)'; 0 0 0 1];

x_bt_hat_final_p = T_b_t_hat_final(1:3,4);
x_bt_hat_final_R = rotm2eul(T_b_t_hat_final(1:3,1:3),'zyx');

x_bt_hat_final = [x_bt_hat_final_p' x_bt_hat_final_R];

disp(T_b_t_hat_final);

% Norm's difference between the initial estimative and the result after the
% Newton's Method (in centimeter)

norm_diff = (norm(x_bt_p) - norm(x_bt_hat_final_p))*100;

%%
% Position error between initial estimation and final estimation by Newton's Method (in cm)

% T_bt is an initial estimative based on URDF
% diff_Tbt = (T_b_t(1:3,4)-T_b_t_hat_final(1:3,4))*100;

% Here I used the calibrated value (via Teach Pendant)
calib_value = [0.89454 0.008 -0.48292];
diff_Tbt = (calib_value' - T_b_t_hat_final(1:3,4))*100;

% Difference between the points measurement in cm (experiment vs matlab)

diff_Pb = (P_b_meas_kr10 - P_b)*100;     

% Norm of this difference for each calibration point
norm_diff_points = [];
for i=1:size(diff_Pb,2)
   norm_diff_points = [norm_diff_points; norm(diff_Pb(:,i))]; 
end

%% Objective Function

function y = obj_fun(x_bt, P, P_t)

    %n_points = length(P);
    
    p_bt = x_bt(1:3);
    R_bt = eul2rotm(x_bt(4:6),'zyx');
    T_b_t = [R_bt p_bt(:); 0 0 0 1];

    y_matrix = P - T_b_t(1:3,:)*P_t;
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