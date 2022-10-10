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
X = opti.variable(NumBodies*2, N+1); % state [q; qd]
q = X(1:NumBodies, :);
qd = X(NumBodies+1:NumBodies*2, :);

% U = opti.variable(NumBodies, N); % control [tau]
% tau = U(1:NumBodies*1, :);

P = opti.variable(NumBodies*(3+1+3+6)+1, 1); % static parameters
d = P(1:NumBodies*3, 1);
m = P(NumBodies*3+1:NumBodies*4, 1);
CoM = P(NumBodies*4+1:NumBodies*7, 1);
I = P(NumBodies*7+1:NumBodies*13, 1);
g = P(end, 1);

T = 0.5; % final time

% ---- objective ----
opti.minimize(sum(sum(q))); % minimal end state

% ---- dynamic constraints ----
f = CasadiForwardDynamics(robot);

dt = T/N; % length of a control interval
for k=1:N % loop over control intervals
%     opti.subject_to(U(:, k) == zeros(NumBodies, 1)); % specify torque input
%     Uk = cos(dt*k).*ones(NumBodies,1);
    Uk = zeros(NumBodies, 1);
    disp(Uk);
    % Runge-Kutta 4 integration
    k1 = f('x', X(:,k),              'u', Uk, 'p', P);
    k2 = f('x', X(:,k)+dt/2*k1.xdot, 'u', Uk, 'p', P);
    k3 = f('x', X(:,k)+dt/2*k2.xdot, 'u', Uk, 'p', P);
    k4 = f('x', X(:,k)+dt*k3.xdot,   'u', Uk, 'p', P);
    x_next = X(:,k) + dt/6*(k1.xdot+2*k2.xdot+2*k3.xdot+k4.xdot);
    opti.subject_to(X(:,k+1)==x_next); % close the gaps
end

% ---- path constraints ----
% input bound
% opti.subject_to(-10<=U(1,:)<=1);
% opti.subject_to(-10<=U(2,:)<=1);

% generate bounds for parameters
[d_data, m_data, CoM_data, I_data] = loadData(robot);
params = [reshape(d_data,[],1); reshape(m_data,[],1); reshape(CoM_data,[],1); reshape(I_data,[],1)];
bound_lower = params - 0.1*abs(params);
bound_upper = params + 0.1*abs(params);

for i = 1:NumBodies*13
    opti.subject_to(bound_lower(i) <= P(i) <= bound_upper(i));
end

% ---- boundary conditions ----
% initial condition
state_init = .5 - rand(2*NumBodies,1); % randomize initial state
% state_init = zeros(2*NumBodies,1);
opti.subject_to(q(:,1) == state_init(1:NumBodies,1));
opti.subject_to(qd(:,1) == state_init(NumBodies+1:2*NumBodies,1));

% ---- initial values for solver ----
opti.set_initial(P(1:end-1), params);
opti.subject_to(g == 9.8);

% ---- solve nlp ----
opti.solver('ipopt');
sol = opti.solve();

%% ---- plot ----
params_opt = sol.value(P);
% ode result of optimal parameters
[t, y] = ode45(@(t,y) full(f(y, cos(t)*ones(NumBodies, 1), params_opt)), ...
    [0 T], state_init);
% ode result of original parameters
[t1, y1] = ode45(@(t1,y1) full(f(y1, cos(t1)*ones(NumBodies, 1), [params; 9.8])), ...
    [0 T], state_init);
for i = 1:NumBodies
    n = NumBodies;
    subplot(n,2,2*i-1); 
    p = plot(t, y(:, i), 'DisplayName', 'p^*'); hold on; grid on;
    p1 = plot(t1, y1(:, i), '--', 'DisplayName', 'p_0'); title(['$q_', num2str(i), '$'], 'Interpreter', 'latex', 'FontSize', 14);
    subplot(n,2,2*i); 
    plot(t, y(:, n+i)); hold on; grid on;
    plot(t1, y1(:, n+i), '--'); title(['$\dot q_', num2str(i), '$'], 'Interpreter', 'latex', 'FontSize', 14);
end
legend([p, p1]);