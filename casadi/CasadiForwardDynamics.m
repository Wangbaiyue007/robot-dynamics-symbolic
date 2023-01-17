%% Added EulerLagrange Function rewritten to work with Casadi

function f = CasadiForwardDynamics(robot, useJointConstants)
%% Generate Casadi symbolic dynamics equation
% using Euler-Lagrange formulation. All calculations are symbolic.
arguments
    robot rigidBodyTree % robot object
    useJointConstants logical = 0 % whether or not to use the friction,damping and armature
end

import casadi.*

%% Parameters (converted)
Dyn = DynamicsSym(robot);

%% Transformations
[Ti, Tci] = Dyn.TransformationMatrices;

%% Jacobians
[Jv, Jw] = Dyn.Jacobians(Ti, Tci);

%% Equations of motion
D = Dyn.MassMatrix(Ti, Jv, Jw);
C = Dyn.CorioliMatrix(D);
G = Dyn.GravitationMatrix(Tci);
f = Dyn.ForwardDynamics(D, C, G, useJointConstants);


end