classdef KinematicsSym
%Symbolic kinematics class
    methods (Static)
        function R = rotZ(q)
            %Rotation matrix about z axis for q symbolic angle
            R = [cos(q) -sin(q) 0 0;
                 sin(q) cos(q)  0 0;
                 0      0       1 0;
                 0      0       0 1];
        end
        function T = transl(d)
            %Translation for d in 3D
            T = [eye(3) d;
                 0 0 0  1];
        end
        function T = transf(R, d)
            %Translation for d in 3D
            T = [  R    d;
                 0 0 0  1];
        end
        function A = HomogenousTransfDH(dh)
            %DH transforamtion
            %input: dh: [r alpha d theta]
                r = dh(1);
                alpha = dh(2);
                d = dh(3);
                theta = dh(4);
                A = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) r*cos(theta);
                     sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) r*sin(theta);
                     0          sin(alpha)             cos(alpha)            d;
                     0          0                      0                     1];
        end
    end
end