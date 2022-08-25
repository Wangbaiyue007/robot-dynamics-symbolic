function [D, C, G] = EulerLagrange(robot, DH)
%% Generate symbolic dynamics equation
% using Euler-Lagrange formulation. In this function only the joint states
% are symbolic. Other parameters are treated as real numbers.
arguments
    robot rigidBodyTree % robot object
    DH double % matrices of the DH parameters with 4 columns
end

%% Parameters
N = robot.NumBodies;
q = sym('q', [N 1], 'real'); % generalized coordinates (joint angles)
qd = sym('qd', [N 1], 'real'); % q's derivative w.r.t time
g = 9.8;

%% Transformations
% transformation matrix Ti is a (N by 1) cell, each cell is a (4 by 4)
% symbolic matrix that transform from 0 to i.
Ti = cell(N, 1);
homeconfiguration = homeConfiguration(robot);
Ti{1} = getTransform(robot, homeconfiguration, robot.BaseName, robot.BodyNames{1}) * RotZ(q(1));
T = Ti{1};
for i = 2:N
    tf = getTransform(robot, homeconfigurationw, robot.BodyNames{i-1}, robot.BodyNames{i}) * RotZ(q(i));
    T = T*tf;
    Ti{i} = T;
end

%% Jacobians
% angular velocity Jacobian Jw (1 by N) cell, each cell is a (3 by N)
% symbolic matrix
Jw = arrayfun(@(x) sym(['Jw' num2str(x)], [3,N], 'real'), 1:N, 'UniformOutput', 0);
% initialize the Jacobian for the first joint and calculate the following
for i = 1:N
    Jw{i} = sym([[0;0;1] repmat([0;0;0], 1, N-1)]);
end
for link = 2:N
    for i = 2:link
        R = Ti{i-1}(1:3,1:3);
        Jw{link}(:, i) = sym(R*[0;0;1]);
    end
end

% velocity Jacobian Jv (1 by n) cell, each cell is a (3 by n) symbolic matrix
Jv = arrayfun(@(x) sym(['Jv' num2str(x)], [3,N], 'real'), 1:N, 'UniformOutput', 0);
for link = 1:N
    o = Ti{link}(1:3, 4);
    for joint = 1:N
        Jv{link}(:, joint) = diff(o, q(joint));
    end
end

%% Inertia tensor
% I (1 by N) cell array, each cell is a (3 by 3) double matrix
I = arrayfun(@(joint) InertiaTensor(robot.Bodies{joint}.Inertia), 1:N, 'UniformOutput', 0);

%% Equations of motion
% D (N by N) symbolic matrix, C (N by N) symbolic matrix
D = 0; % inertia matrix
PE = 0; % potential energy
for i = 1:N
    R = Ti{i}(1:3,1:3);
    D = D + (robot.Bodies{i}.Mass*Jv{i}'*Jv{i} + Jw{i}'*R*I{i}*R'*Jw{i});
    PE = PE + robot.Bodies{i}.Mass * g * Ti{i}(3, 4);
end

% The Christoffel symbols
c = zeros(N, N, N,'sym');

% The Coriolis matrix
C = zeros(N,N,'sym');
for k = 1:N
    for j = 1:N
        for i = 1:N
            c(i,j,k) = 1/2 * (diff(D(k,j),q(i)) + diff(D(k,i),q(j)) - diff(D(i,j),q(k)));
            C(j,k) = C(j,k) + c(i,j,k)*qd(i);
        end
    end
end

% The gravitation terms
G = zeros(N,1,'sym');
for i = 1:N
    G(i) = diff(PE,q(i));
end

end