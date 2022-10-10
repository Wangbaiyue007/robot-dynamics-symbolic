%% Added EulerLagrange Function rewritten to work with Casadi

function f = CasadiForwardDynamics(robot)
%% Generate Casadi symbolic dynamics equation
% using Euler-Lagrange formulation. All calculations are symbolic.
arguments
    robot rigidBodyTree % robot object
end

import casadi.*

%% Parameters (converted)
N = robot.NumBodies;
d = SX.sym('d', N, 3); % sym('d', [N 3], 'real'); % symbolic translation 
m = SX.sym('m', N, 1); % sym('m', [N 1], 'real'); % mass
CoM = SX.sym('c', N, 3); % sym('c', [N 3], 'real'); % center of mass offset
I = SX.sym('I', N, 6); % sym('I', [N 6], 'real'); % inertia vector
q = SX.sym('q', N, 1); % sym('q', [N 1], 'real'); % generalized coordinates (joint angles)
qd = SX.sym('qd', N, 1); % sym('qd', [N 1], 'real'); % q's derivative w.r.t time

% R = arrayfun(@(x) sym('R', [3 3], 'real'), 1:N, 'UniformOutput', 0); % rotation matrix
g = SX.sym('g');

%% Transformations
% Transformation matrix Ti is a (N by 1) cell, each cell is a (4 by 4)
% symbolic matrix that transform from coordinates 0 to i. Tci is the
% transformation from center of mass 0 to i
k = KinematicsSym;
home = homeConfiguration(robot);
Ti = arrayfun(@(x) SX.zeros(4,4), 1:N, 'UniformOutput',0); % arrayfun(@(x) sym(zeros(4, 4)), 1:N, 'UniformOutput', 0);
R = getTransform(robot, home, robot.BodyNames{1}, robot.BaseName);
R = round(R(1:3,1:3)); % round to nearest integer, either 0 or +-1
% not sure if it is necessary to rewrite k.transf and k.rotz etc
Ti{1} = k.transf(R, d(1,:)') * k.rotZ(q(1));
T = Ti{1};
for i = 2:N
    R = getTransform(robot, home, robot.BodyNames{i}, robot.BodyNames{i-1});
    R = round(R(1:3,1:3));
    tf = k.transf(R, d(i,:)') * k.rotZ(q(i));
    T = T * tf;
    Ti{i} = T;
end
Tci = arrayfun(@(x) SX.zeros(4,4), 1:N, 'UniformOutput', 0); % arrayfun(@(x) sym(zeros(4, 4)), 1:N, 'UniformOutput', 0);
for i = 1:N
    Tci{i} = Ti{i} * k.transl(CoM(i,:)');
end

%% Jacobians
% angular velocity Jacobian Jw (1 by N) cell, each cell is a (3 by N)
% symbolic matrix
Jw = arrayfun(@(x) SX.zeros(3,N), 1:N, 'UniformOutput', 0); % arrayfun(@(x) sym(zeros(3, N)), 1:N, 'UniformOutput', 0);
for link = 1:N
    for i = 1:link
        R_i = Ti{i}(1:3,1:3);
        Jw{link}(:, i) = R_i*[0;0;1]; % sym(R_i*[0;0;1]);
    end
end

% velocity Jacobian Jv (1 by n) cell, each cell is a (3 by n) symbolic matrix
Jv = arrayfun(@(x) SX.zeros(3,N), 1:N, 'UniformOutput', 0); % arrayfun(@(x) sym(zeros(3, N)), 1:N, 'UniformOutput', 0);
for link = 1:N
    on = Tci{link}(1:3, 4); % end effector position
    for i = 1:link
        oi_1 = Ti{i}(1:3, 4); % joint position
        Jv{link}(:, i) = cross(Jw{N}(:, i), on - oi_1);
    end
end

%% Inertia tensor
% I_tensor (1 by N) cell array, each cell is a (3 by 3) symbolic matrix
I_tensor = arrayfun(@(joint) DynamicsSym.InertiaTensor(I(joint, :)), ...
            1:N, 'UniformOutput', 0);

%% Equations of motion
% D (N by N) symbolic matrix, C (N by N) symbolic matrix
D = SX.zeros(N, N); % inertia matrix
P = SX.zeros(1); % potential energy

for i = 1:N
    R = Ti{i}(1:3,1:3);
    D = D + (m(i)*Jv{i}'*Jv{i} + Jw{i}'*R*I_tensor{i}*R'*Jw{i});
    P = P + m(i) * g * Tci{i}(3, 4);
end

% The Coriolis matrix
C = SX.zeros(N, N);
for k = 1:N
    for j = 1:N
        for i = 1:N
            c = 1/2 * (jacobian(D(k,j),q(i)) + jacobian(D(k,i),q(j)) - jacobian(D(i,j),q(k)));
            C(k, j) = C(k, j) + c*qd(i);
        end
    end
end

% The gravitation terms
G = SX.zeros(N,1);
for i = 1:N
    G(i) = jacobian(P,q(i));
end

tau = SX.sym('tau', N, 1);
% Define state and input
X = [q; qd];
U = tau;
P = [reshape(d,3*N,1); m; reshape(CoM,3*N,1); reshape(I,6*N,1); g];

qdd = D\(- C * qd - G + tau);
% Xdot = [qd; qdd; zeros(N*13+1,1)];
Xdot = [qd; qdd];
f = Function('f', {X, U, P}, {Xdot}, ...
            {'x', 'u', 'p'}, {'xdot'});


end