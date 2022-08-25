clear; clc; close all;

%% load robot parameters and urdf
DH = gen3_DH();
robot = importrobot('models/urdf/gen3.urdf');
robot.Gravity = [0 0 -9.8];
robot.DataFormat = 'column';
params = gen3_robModel();
fs = params(:, end-2:end);

%% generate dynamics equations
[D, C, G] = EulerLagrange(robot, DH);

%% forward dynamics
q = rand(7,1);
qd = rand(7,1);
tau = rand(7,1);
qdd1 = ForwardDynamicsSym(D, C, G, q, qd, tau);
qdd2 = forwardDynamics(robot, q, qd, tau);
diff = qdd1 - qdd2;
disp(diff);