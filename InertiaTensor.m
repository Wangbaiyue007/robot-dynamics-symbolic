function I = InertiaTensor(inertia_vec)
%Generate inertia tensor from input vector
%   input: [Ixx, Iyy, Izz, Iyz, Ixz, Ixy]
Ixx = inertia_vec(1);
Iyy = inertia_vec(2);
Izz = inertia_vec(3);
Iyz = inertia_vec(4);
Ixz = inertia_vec(5);
Ixy = inertia_vec(6);

I = [Ixx, Ixy, Ixz;
     Ixy, Iyy, Iyz;
     Ixz, Iyz, Izz];
end

