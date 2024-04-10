close all
clear
clc

set(groot,'defaulttextinterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

addpath("rvctools\")
startup_rvc;

%% PANDA ROBOT
% https://frankaemika.github.io/docs/control_parameters.html

flange = false;

a = [0, 0, 0, 0.0825, -0.0825, 0, 0.088];
d = [0.333, 0, 0.316, 0, 0.384, 0, 0];
alpha = [0, -pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2];
theta = [0,0,0,0,0,0,0];

% theta d a alpha
flange_row = [0, 0.107,0,0];
panda_DH = [ theta' d' a' alpha'];
if(flange)
    panda_DH(8,:) = flange_row;
end

panda = SerialLink(panda_DH,'modified');
panda.name = 'PANDA';
panda.offset = zeros(1,size(panda_DH,1));


if(flange)
    q_max = [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159, pi];
    q_min = [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159, -pi];
else
    q_max = [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159];
    q_min = [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159];
end

panda.qlim(:,1) = q_min;
panda.qlim(:,2) = q_max;


%% TEST 7 DoF
% Joint-Space Trajectory
zeroPosition = [0, 0, 0, 0, 0, 0, 0];
dtStep = 10;  % Trajectory accuracy, the higher it is the more accurate it is
t = 0:0.05:3;

finalPosition = [0.0, 1.0, 0.0, -1.0, 0.0, 3.0, 0.0];

% Trapezoidal trajectory
[Q,DQ,DDQ] = mtraj(@lspb,zeroPosition,finalPosition,dtStep); % output: pos vel acc
panda.plot(Q)
