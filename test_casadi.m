clear; clc; close all;

%% load robot parameters and urdf
robot = importrobot('models/urdf/kinova_with_dumbbell.urdf');
robot.Gravity = [0 0 -9.8];
robot.DataFormat = 'column';
Dyn = DynamicsSym(robot);
N_act = Dyn.N - Dyn.N_fixed;

%% generate dynamics equations
disp("Generating forward dynamics function...")
tic
f = Dyn.RobotForwardDynamics;
toc

%% evauation
q = randomConfiguration(robot);
qd = 0.5 - rand(N_act,1);
tau = 0.5 - rand(N_act,1);
[d, m, CoM, I] = loadData(robot);
g = 9.8;
disp("Evaluating forward dynamics...")
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

%% calculate acceleration of the last link
[Ti, Tci] = Dyn.TransformationMatrices;
[Jv, Jw] = Dyn.Jacobians(Ti, Tci);
alpha = Dyn.rotm2eul(Tci{7}, 'ZYZ'); % convert rot matrix to euler angles
Ja = Dyn.AnalyticJacobian(Jv{7}, Jw{7}, alpha); % analytic Jacobian
disp("Generating task space acceleration function...")
tic
f_x_ddot = Dyn.TaskspaceAcc(Ja);
toc
disp("Evaluating task space acceleration...")
tic
x_ddot = f_x_ddot('states', [q; qd; qdd_sym], ...
    'p', [reshape(d,[],1);m;reshape(CoM,[],1)]);
toc
disp(x_ddot.x_ddot);
