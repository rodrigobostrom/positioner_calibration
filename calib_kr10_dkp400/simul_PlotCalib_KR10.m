%% Calibration Plots

simul_TestCalib_KR10;

%%
% Plots da posição
II = ones(1,stop_iter);
III = ones(3,stop_iter);

x_true = II*T_b_t(1,4);
y_true = II*T_b_t(2,4);
z_true = II*T_b_t(3,4);


x_hat = X(:,1);
y_hat = X(:,2);
z_hat = X(:,3);

X_true = III.*T_b_t(1:3,4);
% X_true = calib_value';
X_hat = X(:,1:3)';

dX_pos = 1000*(X_true - X_hat);

%%
figure;
plot(I,dX_pos);
title('Position Error');
legend('X', 'Y', 'Z');
ylabel('Position Error [mm]');
xlabel('Iteration');
grid

%%
% % Plots da orientação
% figure(2)
% subplot(3,1,1) 
% plot(I,X(4,:),I,phi_hat*ones(1,stop_iter))
% ylabel('Phi')
% xlabel('Iterações')
% legend('phi_{bt}', 'phi_{hat}')
% title('Orientação(rad)')
% grid
% subplot(3,1,2) 
% plot(I,X(5,:),I,theta_hat*ones(1,stop_iter))
% ylabel('Theta')
% xlabel('Iterações')
% legend('theta_{bt}', 'theta_{hat}')
% grid
% subplot(3,1,3) 
% plot(I,X(6,:),I,psi_hat*ones(1,stop_iter))
% ylabel('Psi')
% xlabel('Iterações')
% legend('psi_{bt}', 'psi_{hat}')
% grid

%%
% Norma da função objetivo
figure
% plot(I, N, I, zeros(1,stop_iter));
plot(I, N);
% grid on;
title('Norm of Objective Function');
xlabel('Iteration');
ylabel('Norm');
legend('Norm');
grid

%%
pause(1);
figure;
A = P_b;
plot3(A(1,:),A(2,:),A(3,:),'*'); hold on;
A = P_b_meas_kr10;
plot3(A(1,:),A(2,:),A(3,:),'*');
axis equal
trplot(eye(4),'frame','Base Robot','length',0.5);
trplot(T_b_t,'frame','Table','length',0.5);
trplot(T_b_t*T_t_d.T,'frame','Depos. Frame','length',0.5);
grid

for i = I
    x_bt_i = X(i,:);
    T_hat_i = xyzeul2tform(x_bt_i);
    
    h1 = trplot(T_hat_i,'color','r','length',0.5,'frame','Estimative');
    title(['Iteration ' num2str(i) '/' num2str(I(end))]);
    if i == 1
        pause
    end
    pause(0.1);
    
    if (i ~= I(end))
        set(h1,'Visible','off');
    end

end