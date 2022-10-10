% Finding maximum/minimum trajectory by selecting parameters
% ----------------------
% An optimal control problem (OCP),
% solved with direct multiple-shooting.
clear; close all; clc;

N = 100; % number of control intervals
opti = casadi.Opti(); % Optimization problem

% ---- load robot ----
robot = importrobot('models/urdf/2link.urdf');
robot.Gravity = [0 0 -9.8];
robot.DataFormat = 'column';
NumBodies = robot.NumBodies;

% ---- decision variables ----
X = opti.variable(NumBodies*2, N+1); % [q; qd; d; m; CoM; I; g]
q = X(1:NumBodies, :);
qd = X(NumBodies+1:NumBodies*2, :);

U = opti.variable(NumBodies*(1+3+1+3+6)+1, N); % control [tau; d; m; CoM; I; g]
tau = U(1:NumBodies*1, :);
d = U(NumBodies*1+1:NumBodies*4, :);
m = U(NumBodies*4+1:NumBodies*5, :);
CoM = U(NumBodies*5+1:NumBodies*8, :);
I = U(NumBodies*8+1:NumBodies*14, :);
g = U(end, :);

T = 1; % final time

% ---- objective ----
opti.minimize(sum(sum(q))); % minimal end state

% ---- dynamic constraints ----
f = CasadiForwardDynamics(robot);

dt = T/N; % length of a control interval
for k=1:N % loop over control intervals
%     opti.subject_to(U(:, k) == cos(dt*k)*ones(NumBodies, 1)); % specify torque input
    % Runge-Kutta 4 integration
    k1 = f('x', X(:,k),              'u', U(:,k));
    k2 = f('x', X(:,k)+dt/2*k1.xdot, 'u', U(:,k));
    k3 = f('x', X(:,k)+dt/2*k2.xdot, 'u', U(:,k));
    k4 = f('x', X(:,k)+dt*k3.xdot,   'u', U(:,k));
    x_next = X(:,k) + dt/6*(k1.xdot+2*k2.xdot+2*k3.xdot+k4.xdot);
    opti.subject_to(X(:,k+1)==x_next); % close the gaps
end

% ---- path constraints ----
% input bound
% opti.subject_to(0<=U(1,:)<=1);
% opti.subject_to(0<=U(2,:)<=1);

% generate bounds for parameters
[d_data, m_data, CoM_data, I_data] = loadData(robot);
params = [reshape(m_data,[],1); reshape(CoM_data,[],1); reshape(I_data,[],1)];
bound_lower = params - 0.03;
bound_upper = params + 0.03;

for i = 1:NumBodies*10
    opti.subject_to(U(NumBodies+i,:) <= bound_upper(i));
    opti.subject_to(U(NumBodies+i,:) >= bound_lower(i));
end

% ---- boundary conditions ----
% initial condition
% state_init = .5 - rand(2*NumBodies,1); % randomize initial state
state_init = zeros(2*NumBodies,1);
opti.subject_to(q(:,1) == state_init(1:NumBodies,1));
opti.subject_to(qd(:,1) == state_init(NumBodies+1:2*NumBodies,1));
opti.subject_to(d(:,1) == reshape(d_data,[],1));
opti.subject_to(g(1) == 9.8);


% ---- initial values for solver ----
opti.set_initial([m(:,1);CoM(:,1);I(:,1)], params);

% ---- solve nlp ----
opti.solver('ipopt');
sol = opti.solve();
