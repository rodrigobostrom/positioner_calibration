function arm = KR90_URDF_SerialLink()
%% This SerialLink does not need a remap between joints from ROS and MATLAB
%
% theta = vector_from_joint_states;
% T = arm.fkine(theta)
%
% T is equal to /kr90/endpoint_state



%% DH
% Parameters (meters and radians)
a1 = 0.350;
a2 = 1.350;
a3 = 0.041;
alpha1 = pi/2;
alpha3 = -pi/2;
alpha4 = pi/2;
alpha5 = -pi/2;
alpha6 = pi;
d0 = 0.675+0.4636*0; % 0.675+0.4636 -> old base offset (not a tf anymore)
d4 = -1.200;
d6 = -0.215;
offset3 = pi/2;


% Links
L(1) = Link('d',    0,  'a',    a1, 'alpha',    alpha1, 'offset', 0         );
L(2) = Link('d',    0,  'a',    a2, 'alpha',    0,      'offset', 0         );
L(3) = Link('d',    0,  'a',    a3, 'alpha',    alpha3, 'offset', offset3   );
L(4) = Link('d',    d4, 'a',    0,  'alpha',    alpha4, 'offset', 0         );
L(5) = Link('d',    0,  'a',    0,  'alpha',    alpha5, 'offset', 0         );
L(6) = Link('d',    d6, 'a',    0,  'alpha',    alpha6, 'offset', 0         );


x = [1;0;0]; y = [0;1;0]; z = [0;0;1];
Rbase = [x -y -z]*rotz(-90*0); % rotz(-90) added, please remove it later
pbase = [0; 0; d0];
tbase0 = [Rbase pbase; 0 0 0 1]; % To match URDF and TeachPendant


xyz = [0.00629 -0.00465 0.43388];
rpy = deg2rad([4.20 43.9 -25.10]);
t6e = urdf2tform([xyz rpy]);
% t6e = eye(4);


 %% TF to profil ## CHANGE REFERENCE HEIGHT ##
% t6e = t6e*urdf2tform([0.000 0.150+0.015 -0.175 -0.05 0.00 3.0676-pi]); % profil
% t6e = t6e*urdf2tform([0.055 0.180 -0.175 -0.05 0.00 3.0676-pi]); % profil 28/feb
% t6e = t6e*urdf2tform([0.085 0.165 -0.150 -0.05 0.00 3.0676-pi]); % profil 06/may

%%
% t6e = trotz(180);

%% TF to pen tip
% t6e = t6e*transl([-0.020;0;-0.0015]);
% t6e = t6e*transl([-0.017; 0; 0.015]); % 28/fev


%% TF to ponta do arame (para fazer inclinado) 09/may/2022
% t6e = t6e*transl([0; 0; 15/1000]);


%%
% Raw
arm = SerialLink(L);
arm.base = tbase0;
arm.tool = t6e;
arm.name = 'KR90';


%% Notes
% base = '/kr90_base_link'
% tool = '/kr90_tool0'

