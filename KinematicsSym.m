classdef KinematicsSym
%Symbolic kinematics class
    methods (Static)
        function R = rotX(q)
            %Rotation matrix about y axis for q symbolic angle
            R = [1  0       0      0;
                 0  cos(q) -sin(q) 0;
                 0  sin(q) cos(q)  0;
                 0  0       0      1];
        end
        function R = rotY(q)
            %Rotation matrix about y axis for q symbolic angle
            R = [cos(q)  0 sin(q) 0;
                 0       1 0      0;
                 -sin(q) 0 cos(q) 0;
                 0       0 0      1];
        end
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
        function T = rpy(angles)
            %Roll, pitch, yaw transformation matrix
            phi = angles(3);
            theta = angles(2);
            psi = angles(1);
            Rz = KinematicsSym.rotZ(phi);
            Ry = KinematicsSym.rotY(theta);
            Rx = KinematicsSym.rotX(psi);
            T = Rz*Ry*Rx;
        end
        function T = urdfTransf(rpy, xyz)
            %Transformation using urdf data
            T1 = KinematicsSym.transl(xyz');
            T2 = KinematicsSym.rpy(rpy);
            T = T1 * T2;
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