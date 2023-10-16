function table = KP2_URDF_SerialLink()
%% DH
% Parameters (meters and radians)
alpha1 = pi/2; % -pi/2
d0 = 0.865;
d2 = 0.220;
offset2 = -pi/2; % 0


% Links
L(1) = Link('d',    0,  'a',    0,  'alpha',    alpha1, 'offset', 0         );
L(2) = Link('d',    d2, 'a',    0,  'alpha',    0,      'offset', offset2   );


x = [1;0;0]; y = [0;1;0]; z = [0;0;1];
Rbase = [y -z -x];
pbase = [0; 0; d0];
tbase0 = [Rbase pbase; 0 0 0 1]; % To match URDF

% t6e = eye(4);


%%
table = SerialLink(L);
table.base = tbase0;
% table.tool = t6e;
table.name = 'Table';


%% Notes
% base = '/kp2_base_link'
% tool (link2) = '/kp2_tool0'

