clear; clc; close all;

%% load robot parameters and urdf
robot = importrobot('models/urdf/gen3.urdf');
robot.Gravity = [0 0 -9.8];
robot.DataFormat = 'column';

%% generate dynamics equations
tic
f = CasadiEulerLagrange(robot);
toc

%% evauation
q = randomConfiguration(robot);
qd = 0.5 - rand(robot.NumBodies,1);
tau = 0.5 - rand(robot.NumBodies,1);
[d, m, CoM, I] = loadData(robot);
tic
qdd_sym = f('q', q, 'qd', qd, 'tau', tau, 'd', d, 'm', m, 'CoM', CoM, 'I', I, 'g', 9.8);
toc
qdd_real = forwardDynamics(robot, q, qd, tau);
error = qdd_sym.qdd - qdd_real;
disp('   qdd_sym   qdd_real');
disp([qdd_sym.qdd qdd_real]);
disp("Normalized error:");
disp(norm(error)/norm(qdd_real));