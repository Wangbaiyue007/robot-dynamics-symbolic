%% Simulation of rigid links

clc;
clear;
close all;

%% Parameters

% x0_s=[0.0 0. 0 0.0 0.0 0 zeros(1,6)];
x0_s=[0.2 0.2 1 0.2 0.2 0.1 zeros(1,6)]; % initial condition for state vector
k_spring=[100 10 10]; % spring constant

%% Built-in Link Properties, calculate offline

I1=10;  I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1; 
g=9.8;
[D, C] = EquationsOfMotion(m1,r1,l1); % this will take time

%% Simulation Process

tf=0.35;
params = {k_spring};
options = odeset('RelTol',1e-4,'AbsTol',[1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2]);
[T,W] = ode113(@(t,w) SystemDynamics(t,w,D,C,params),[0 tf],x0_s, options);

%% Plot

figure(1);
subplot(3, 1, 1);
plot(T, W(:,1),'r-');
title('Theta1 Evolution, joint 1')
xlabel('Time')
ylabel('Theta1')
hold on
grid on
% 
subplot(3, 1, 2);
plot(T, W(:,2),'r-');
title('Theta2 Evolution, joint 1')
xlabel('Time')
ylabel('Theta2')
hold on
grid on
%
subplot(3, 1, 3);
plot(T, W(:,3),'r-');
title('Theta3 Evolution, joint 1')
xlabel('Time')
ylabel('Theta3')
hold on
grid on

figure(2);
subplot(3, 1, 1);
plot(T, W(:,4),'r-');
title('Theta1 Evolution, joint 2')
xlabel('Time')
ylabel('Theta1')
hold on
grid on
% 
subplot(3, 1, 2);
plot(T, W(:,5),'r-');
title('Theta2 Evolution, joint 2')
xlabel('Time')
ylabel('Theta2')
hold on
grid on
%
subplot(3, 1, 3);
plot(T, W(:,6),'r-');
title('Theta3 Evolution, joint 2')
xlabel('Time')
ylabel('Theta3')
hold on
grid on
