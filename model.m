%%% Titus Braber - Vision-based stabilization of micro quadrotors
%%% 2D simulation of a quad

%% Make the symbolic variables
syms m w h I g positive
syms x xd z zd p pd t F1 F2 F M real
symbolic = [m;w;h;I;g];

%% Create the EOM function
% Define the state vector
q = [x;z;p;xd;zd;pd];

% Define the inputs
F = F1+F2;
M = F1*w/2 - F2*w/2;

% Define the equations of motion
xdd = (F * sin(p) )/m;
zdd = (F * cos(p) - g * m)/m;
pdd = M/I;

% Define the state derivative vector & input vector
qd = [xd;zd;pd;xdd;zdd;pdd];
U = [F1;F2];

funcInput = [q;U];
global control;