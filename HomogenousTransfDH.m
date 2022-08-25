function [A_Frame] = HomogenousTransfDH(dh)
%input: dh: [r alpha d theta]
    r = dh(1);
    alpha = dh(2);
    d = dh(3);
    theta = dh(4);
    A_Frame = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) r*cos(theta);
               sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) r*sin(theta);
               0          sin(alpha)             cos(alpha)            d;
               0          0                      0                     1];
end