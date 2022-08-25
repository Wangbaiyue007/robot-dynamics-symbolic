%% Virtual Model controller (spring-mass), with only proportional control


function dx = SystemDynamics(t,x, D, C,params)
% x = [q; qdot] a 2n by 1 vector

%% User parameters that should be tuned later
% Getting K_P and K_D as a positive definite, symmetric, diagonal matrix
% from the parameters:
K_spring = params{1};
K_1 = K_spring(1);
K_2 = K_spring(2);
K_3 = K_spring(3);

%% Built-in physics, no need to worry
% Measurements of the 2-link arm
Cval = C(x(1),x(2),x(3),x(4),x(5),x(6),x(7),x(8),x(9),x(10),x(11),x(12));
Dval = D(x(1),x(2),x(3),x(4),x(5),x(6));

%% Connection of n rigid links by spherical joints

u = zeros(6,1);
for i = 1:6
    u(i) = K_1 *(0 - x(i));
end

%% Simulation process calculated by ode. No need to worry.

% From state-space representation, dx = [q_dot q_double_dot]^T
dx = zeros(12,1);

% q_dot can be taken from the current state variable x
q_dot = x(7:12);
dx(1:6) = q_dot;

% q_double_dot comes from the dynamics of the arm and setting the torque as
% the input controller u
q_double_dot = Dval \ (u - Cval*q_dot - 0);
dx(7:12) = q_double_dot;

end