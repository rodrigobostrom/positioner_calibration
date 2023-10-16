function arm = KR30_URDF_SerialLink()
%% This SerialLink does not need a remap between joints from ROS and MATLAB
%
% theta = vector_from_joint_states;
% T = arm.fkine(theta)
%
% T is equal to /kr30/endpoint_state



%% DH
% Parameters (meters and radians)
a1 = 0.175;
a2 = 0.890;
a3 = -0.050;
alpha1 = pi/2;
alpha3 = -pi/2;
alpha4 = pi/2;
alpha5 = -pi/2;
alpha6 = pi;
d0 = 0.575;
d4 = -1.035;
d6 = -0.185;
offset3 = pi/2;


% Links
L(1) = Link('d',    0,  'a',    a1, 'alpha',    alpha1, 'offset', 0         );
L(2) = Link('d',    0,  'a',    a2, 'alpha',    0,      'offset', 0         );
L(3) = Link('d',    0,  'a',    a3, 'alpha',    alpha3, 'offset', offset3   );
L(4) = Link('d',    d4, 'a',    0,  'alpha',    alpha4, 'offset', 0         );
L(5) = Link('d',    0,  'a',    0,  'alpha',    alpha5, 'offset', 0         );
L(6) = Link('d',    d6, 'a',    0,  'alpha',    alpha6, 'offset', 0         );


x = [1;0;0]; y = [0;1;0]; z = [0;0;1];
Rbase = [x -y -z];
pbase = [0; 0; d0];
tbase0 = [Rbase pbase; 0 0 0 1]; % To match URDF and TeachPendant


% xyz = [0.00629 -0.00465 0.43388]; %% KR90's Fronius torch
% rpy = deg2rad([4.20 43.9 -25.10]);
% t6e = urdf2tform([xyz rpy]);
t6e = eye(4);


%%
% Raw
arm = SerialLink(L);
arm.base = tbase0;
arm.tool = t6e;
arm.name = 'KR30';


%% Notes
% base = '/kr30_base_link'
% tool = '/kr30_tool0'

