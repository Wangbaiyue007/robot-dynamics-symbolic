clear; clc; close all;

%% load robot parameters and urdf
robot = importrobot('models/urdf/2link.urdf');
robot.Gravity = [0 0 -9.8];
robot.DataFormat = 'column';

%% generate dynamics equations
tic
[D, C, G] = EulerLagrange(robot);
toc

%% save as matlab function
dyn = DynamicsSym(robot);
disp("saving function...");
tic
dyn.SaveFunction(D, C, G, 'matlabfunctions/two_link');
toc

%% forward dynamics
q = randomConfiguration(robot);
qd = 0.5 - rand(robot.NumBodies,1);
tau = 0.5 - rand(robot.NumBodies,1);
tic
[qdd_sym, ~, ~, ~] = dyn.ForwardDynamics(robot, D, C, G, q, qd, tau);
toc
qdd_real = forwardDynamics(robot, q, qd, tau);
error = qdd_sym - qdd_real;
disp('   qdd_sym   qdd_real');
disp([qdd_sym qdd_real]);
disp("Normalized error:");
disp(norm(error)/norm(qdd_real));