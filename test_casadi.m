clear; clc; close all;

%% load robot parameters and urdf
robot = importrobot('models/urdf/kinova_with_dumbbell.urdf');
robot.Gravity = [0 0 -9.8];
robot.DataFormat = 'column';
Dyn = DynamicsSym(robot);
N_act = Dyn.N - Dyn.N_fixed;

%% generate dynamics equations
tic
f = Dyn.RobotForwardDynamics;
toc

%% evauation
q = randomConfiguration(robot);
qd = 0.5 - rand(N_act,1);
tau = 0.5 - rand(N_act,1);
[d, m, CoM, I] = loadData(robot);
g = 9.8;
tic
x_dot = f('x', [q;qd], 'u', tau, ...
    'p', [reshape(d,[],1);m;reshape(CoM,[],1);reshape(I,[],1);g]);
qdd_sym = x_dot.xdot(N_act+1:N_act*2);
toc
qdd_real = forwardDynamics(robot, q, qd, tau);
error = qdd_sym - qdd_real;
disp('   qdd_sym   qdd_real');
disp([qdd_sym qdd_real]);
disp("Normalized error:");
disp(norm(error)/norm(qdd_real));

%% calculate force of the last link
