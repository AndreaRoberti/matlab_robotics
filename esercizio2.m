close all
clear
clc

set(groot,'defaulttextinterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

addpath("rvctools\")
startup_rvc;

%% Es 2 RPR 3 DOF
d1 = 1;
a3 = 1;

% create links using D-H parameters
% theta', d', a', alpha'
A0 = Revolute('d',d1,'a',0,'alpha',-pi/2);
A1 = Prismatic('theta',-pi/2,'a',0,'alpha',-pi/2);
A3 = Revolute('d',0,'a',a3,'alpha',0);

A0.qlim = [-pi pi];
A1.qlim = [0 2]; 
A3.qlim = [-pi 0];

% Build the robot model
robot = SerialLink([A0 A1 A3], 'name', 'robot');
% robot.offset = [0 0 0];  

% robot.teach

% Forward Kinematics
finalPosition = [pi/2 0.5 -pi/3];
fkine_pose = robot.fkine(finalPosition);
disp('Forward Kinematics: ');
disp(fkine_pose);
robot.plot(finalPosition)

% Inverse Kinematics 
[joint_variables, error] = robot.ikcon(fkine_pose,[0 0 0]);
disp('Inverse Kinematics: ');
disp(joint_variables);
disp('Error: ');
disp(error);

% Joint-Space Trajectory
zeroPosition = [0 0 0];
dtStep = 50;  % Trajectory accuracy, the higher it is the more accurate it is
t = 0:0.05:2;

% Trapezoidal trajectory
[Q,DQ,DDQ] = mtraj(@lspb,zeroPosition,finalPosition,dtStep); % output: pos vel acc
robot.plot(Q)

X = [];
Y = [];
Z = [];
for i = 1:length(Q)
    fkine_pose = robot.fkine([Q(i,1) Q(i,2) Q(i,3)]);
    pose = transl(fkine_pose);  % retrieve px, py and pz only
    X(i) = pose(1);
    Y(i) = pose(2);
    Z(i) = pose(3);
    i = i + 1;
end

figure;
plot3(X(1), Y(1), Z(1), 'g.', 'MarkerSize', 30);  
hold on;
plot3(X, Y, Z, 'b-', 'LineWidth', 2);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Trajectory');
plot3(X(end), Y(end), Z(end), 'r.', 'MarkerSize', 30);  
grid on;

% Polinomial trajectory
% [Q,DQ,DDQ] = mtraj(@tpoly,zeroPosition,finalPosition,dtStep);
% [Q,DQ,DDQ] = jtraj(zeroPosition,finalPosition,t); % equivalent
% robot.plot(Q)

% figure;
% subplot(211);
% plot(Q); grid on;
% legend('$q_1$','$q_2$','FontSize',14);
% title('Joint angles and positions','FontSize',14);
% subplot(212);
% plot(DQ); grid on;
% legend('$\dot{q}_1$','$\dot{q}_2$','FontSize',14);
% title('Joint velocities','FontSize',14);

% 3D Workspace
X = [];
Y = [];
Z = [];
i = 1;
for q1 = -pi:0.2:pi
    for q2 = 0:0.2:2
        for q3 = -pi:0.2:0
            fkine_pose = robot.fkine([q1 q2 q3]);
            pose = transl(fkine_pose);  % retrieve px, py and pz only
            X(i) = pose(1);
            Y(i) = pose(2);
            Z(i) = pose(3);
            i = i + 1;
        end
    end
end

figure;
plot3(0, 0, 0, 'rx', 'MarkerSize', 20); hold on
plot3(X, Y, Z, 'b.', 'LineWidth', 2);
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on
title('Workspace');