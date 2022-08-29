clear; clc; close all;

%% load robot parameters and urdf
robot = importrobot('models/urdf/gen3.urdf');
robot.Gravity = [0 0 -9.8];
robot.DataFormat = 'column';

%% generate dynamics equations
tic
[D, C, G] = EulerLagrange(robot);
toc

%% forward dynamics
q = randomConfiguration(robot);
qd = 0.5 - rand(7,1);
tau = 0.5 - rand(7,1);
tic
dyn = DynamicsSym(robot);
[qdd_sym, ~, ~, ~] = dyn.ForwardDynamics(robot, D, C, G, q, qd, tau);
toc
qdd_real = forwardDynamics(robot, q, qd, tau);
error = qdd_sym - qdd_real;
disp('   qdd_sym   qdd_real');
disp([qdd_sym qdd_real]);
disp("Normalized error:");
disp(norm(error)/norm(qdd_real));