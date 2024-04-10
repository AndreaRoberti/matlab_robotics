close all
clear
clc

set(groot,'defaulttextinterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

addpath("rvctools\")
startup_rvc;

%% Es 1 RP 2 DOF

a1 = 1;

% create links using D-H parameters
% theta', d', a', alpha'
A0 = Revolute('d',0,'a',a1,'alpha',-pi/2);
A1 = Prismatic('theta',0,'a',0,'alpha',0);

A0.qlim = [-pi/2 pi/2];
A1.qlim = [0 2];  % matlab error if not given!                       

% Build the robot model
robot = SerialLink([A0 A1], 'name', 'robot');

% Exercise initial position
% robot.offset = [-pi/4 1];   % on the joint variable

% Free move
% robot.teach

% Forward Kinematics
finalPosition = [pi/3 2];
fkine_pose = robot.fkine(finalPosition);
disp('Forward Kinematics: ');
disp(fkine_pose);
robot.plot(finalPosition)

% Inverse Kinematics 
[joint_variables, error] = robot.ikcon(fkine_pose,[0 0]);
disp('Inverse Kinematics: ');
disp(joint_variables);
disp('Error: ');
disp(error);

% Joint-Space Trajectory
zeroPosition = [0 0];
dtStep = 50;  % Trajectory accuracy, the higher it is the more accurate it is
t = 0:0.05:2;

% Trapezoidal trajectory
[Q,DQ,DDQ] = mtraj(@lspb,zeroPosition,finalPosition,dtStep); % output: pos vel acc
robot.plot(Q)

% Polinomial trajectory
% [Q,DQ,DDQ] = mtraj(@tpoly,zeroPosition,finalPosition,dtStep);
% [Q,DQ,DDQ] = jtraj(zeroPosition,finalPosition,t); % equivalent
% robot.plot(Q)

figure;
subplot(211);
plot(Q); grid on;
legend('$q_1$','$q_2$','FontSize',14);
title('Joint angles and positions','FontSize',14);
subplot(212);
plot(DQ); grid on;
legend('$\dot{q}_1$','$\dot{q}_2$','FontSize',14);
title('Joint velocities','FontSize',14);

% dtStep = 20;
% figure;
% for i = 1:20
%     [Q,QD,QDD] = jtraj(zeroPosition,finalPosition,dtStep); % output: pos vel acc
%     robot.plot(Q)
%     temp = zeroPosition;
%     zeroPosition = finalPosition;
%     finalPosition = temp;
% end


% Cartesian-Space Trajectory
Xi = [0 0 0];
Xf = [-1.23 1.866 0];
[X, DX] = mtraj(@lspb,Xi,Xf,t);
figure;
subplot(211)
plot(t,[X(:,1) X(:,2)]); grid on;
legend('$x$','$y$','FontSize',14);
title('End-effector position','FontSize',14);
subplot(212)
plot(t,[DX(:,1) DX(:,2)]); grid on;
legend('$\dot{x}$','$\dot{y}$','FontSize',14);
title('End-effector velocity','FontSize',14);

figure;
plot(X(:,1), X(:,2)); grid on;

% 2D Workspace
X = [];
Y = [];
i = 1;
for q1 = -pi/2:0.05:pi/2
    for q2 = 0:0.05:2
        fkine_pose = robot.fkine([q1 q2]);
        pose = transl(fkine_pose);  % retrieve px, py and pz only
        X(i) = pose(1);
        Y(i) = pose(2);
        i = i + 1;
    end
end

figure;
plot(0, 0, 'rx', 'MarkerSize', 20); hold on
plot(X,Y,'.','Color','k'); grid on; grid minor;
xlabel('$X\:[m]$','FontSize',14);
ylabel('$Y\:[m]$','FontSize',14);