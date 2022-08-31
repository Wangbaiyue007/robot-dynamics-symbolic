function [D, C, G] = EulerLagrange(robot)
%% Generate symbolic dynamics equation
% using Euler-Lagrange formulation. All calculations are symbolic.
arguments
    robot rigidBodyTree % robot object
end

%% Parameters
N = robot.NumBodies;
d = sym('d', [N 3], 'real'); % symbolic translation 
m = sym('m', [N 1], 'real'); % mass
CoM = sym('c', [N 3], 'real'); % center of mass offset
I = sym('I', [N 6], 'real'); % inertia vector
q = sym('q', [N 1], 'real'); % generalized coordinates (joint angles)
qd = sym('qd', [N 1], 'real'); % q's derivative w.r.t time
syms g;

%% Transformations
% Transformation matrix Ti is a (N by 1) cell, each cell is a (4 by 4)
% symbolic matrix that transform from coordinates 0 to i. Tci is the
% transformation from center of mass 0 to i
k = KinematicsSym;
home = homeConfiguration(robot);
Ti = arrayfun(@(x) sym(zeros(4, 4)), 1:N, 'UniformOutput', 0);
R = getTransform(robot, home, robot.BodyNames{1}, robot.BaseName);
R = round(R(1:3,1:3)); % round to nearest integer, either 0 or 1
Ti{1} = k.transf(R, d(1,:)') * k.rotZ(q(1));
T = Ti{1};
for i = 2:N
    R = getTransform(robot, home, robot.BodyNames{i}, robot.BodyNames{i-1});
    R = round(R(1:3,1:3));
    tf = k.transf(R, d(i,:)') * k.rotZ(q(i));
    T = T * tf;
    Ti{i} = simplify(T);
end
Tci = arrayfun(@(x) sym(zeros(4, 4)), 1:N, 'UniformOutput', 0);
for i = 1:N
    Tci{i} = simplify(Ti{i} * k.transl(CoM(i,:)'));
end

%% Jacobians
% angular velocity Jacobian Jw (1 by N) cell, each cell is a (3 by N)
% symbolic matrix
Jw = arrayfun(@(x) sym(zeros(3, N)), 1:N, 'UniformOutput', 0);
for link = 1:N
    for i = 1:link
        R = Ti{i}(1:3,1:3);
        Jw{link}(:, i) = sym(R*[0;0;1]);
    end
end

% velocity Jacobian Jv (1 by n) cell, each cell is a (3 by n) symbolic matrix
Jv = arrayfun(@(x) sym(zeros(3, N)), 1:N, 'UniformOutput', 0);
for link = 1:N
    on = Tci{link}(1:3, 4); % end effector position
    for i = 1:link
        oi_1 = Ti{i}(1:3, 4); % joint position
        Jv{link}(:, i) = simplify(cross(Jw{7}(:, i), on - oi_1));
    end
end

%% Inertia tensor
% I_tensor (1 by N) cell array, each cell is a (3 by 3) double matrix
I_tensor = arrayfun(@(joint) DynamicsSym.InertiaTensor(I(joint, :)), ...
            1:N, 'UniformOutput', 0);

%% Equations of motion
% D (N by N) symbolic matrix, C (N by N) symbolic matrix
D = sym(zeros(N, N)); % inertia matrix
P = sym(0); % potential energy
for i = 1:N
    R = Ti{i}(1:3,1:3);
    D = D + (m(i)*Jv{i}'*Jv{i} + Jw{i}'*R*I_tensor{i}*R'*Jw{i});
    P = P + m(i) * g * Tci{i}(3, 4);
end

% The Coriolis matrix
C = sym(zeros(N, N));
for k = 1:N
    for j = 1:N
        for i = 1:N
            c = 1/2 * (diff(D(k,j),q(i)) + diff(D(k,i),q(j)) - diff(D(i,j),q(k)));
            C(k, j) = C(k, j) + c*qd(i);
        end
    end
end

% The gravitation terms
G = sym(zeros(N,1));
for i = 1:N
    G(i) = diff(P,q(i));
end

end