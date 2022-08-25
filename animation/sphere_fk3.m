% ********** File: planar_fk2.m ********** 
% by Jakub Tomasz Kaminski and Baiyue

function [T, pos, z1, z2, z3] = sphere_fk3(th1, th2, th3, l, Tprevious)
    % PLEASE NOTE: 
    arguments
        th1 (1,1)
        th2 (1,1)
        th3 (1,1)
        l (1,1)
        Tprevious (4,4) = eye(4);
    end

    % DH Table values 
    a1 = 0; alpha1 = -pi/2; d1 = 0; theta1 = th1;
    a2 = 0; alpha2 = pi/2;  d2 = 0; theta2 = th2; 
    a3 = l; alpha3 = 0;     d3 = 0; theta3 = th3;

    % Compose a matrix out of the DH values provided
    DH = [a1 alpha1 d1 theta1;
          a2 alpha2 d2 theta2;
          a3 alpha3 d3 theta3];

    A1 = HomogenousTransf(DH(1,:));
    A2 = HomogenousTransf(DH(2,:));
    A3 = HomogenousTransf(DH(3,:));
    A1 = Tprevious*A1;

    z1 = A1(1:3,1:3)*[0;0;1];
    z2 = A2(1:3,1:3)*z1;
    z3 = A3(1:3,1:3)*z2;

    T = A1*A2*A3;
    pos = T(1:3, 4);

end