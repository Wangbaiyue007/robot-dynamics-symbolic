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
q = rand(7,1);
qd = rand(7,1);
tau = rand(7,1);
tic
dyn = DynamicsSym(robot);
[qdd_sym, Dval, Cval, Gval] = dyn.ForwardDynamics(robot, D, C, G, q, qd, tau);
toc
qdd_real = forwardDynamics(robot, q, qd, tau);
diff = qdd_sym - qdd_real;
disp(diff);