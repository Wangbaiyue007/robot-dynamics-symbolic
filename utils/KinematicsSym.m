classdef KinematicsSym
%Symbolic kinematics class
    properties
        robot % matlab robot object
        N % number of bodies
        d % symbolic translation
        CoM % center of mass offset
        q % generalized coordinates (joint angles)
        qd % q's derivative w.r.t time
        N_fixed % number of fixed joints
        fixed_index % array of fixed joints index
        act_index % array of actuated joints index
        q_act % actuated joint angles after removing fixed joints
        qd_act % q_act's derivative w.r.t. time
        alpha % Euler angles
    end
    methods
        function obj = KinematicsSym(robot)
            import casadi.*

            % Initialize Class
            obj.robot = robot;
            obj.N = robot.NumBodies;
            obj.d = SX.sym('d', obj.N, 3);
            obj.CoM = SX.sym('c', obj.N, 3);
            obj.q = SX.sym('q', obj.N, 1);
            obj.qd = SX.sym('qd', obj.N, 1);
            obj.alpha = SX.sym('alpha', 3, 1);

            % properties of fixed joints: positions and velocities remain zero
            obj.N_fixed = 0;
            obj.fixed_index = [];
            obj.act_index = [];
            for i = 1:obj.N
                if robot.Bodies{i}.Joint.Type == "fixed"
                    obj.q(i) = 0;
                    obj.qd(i) = 0;
                    obj.fixed_index = [obj.fixed_index i];
                    obj.N_fixed = obj.N_fixed + 1;
                else
                    obj.act_index = [obj.act_index i];
                end
            end
            obj.q_act = obj.q(obj.act_index); % actuated joints
            obj.qd_act = obj.qd(obj.act_index); % actuated joints rate
        end
        function [Ti, Tci] = TransformationMatrices(obj)
            % Transformation matrix Ti is a (N by 1) cell, each cell is a (4 by 4)
            % symbolic matrix that transform from coordinates 0 to i. Tci is the
            % transformation from center of mass 0 to i
            import casadi.*

            home = homeConfiguration(obj.robot);
            Ti = arrayfun(@(x) SX.zeros(4,4), 1:obj.N, 'UniformOutput',0); % arrayfun(@(x) sym(zeros(4, 4)), 1:N, 'UniformOutput', 0);
            R = getTransform(obj.robot, home, obj.robot.BodyNames{1}, obj.robot.BaseName);
            R = round(R(1:3,1:3)); % round to nearest integer, either 0 or +-1
            % not sure if it is necessary to rewrite k.transf and k.rotz etc
            Ti{1} = obj.transf(R, obj.d(1,:)') * obj.rotZ(obj.q(1));
            T = Ti{1};
            for i = 2:obj.N
                R = getTransform(obj.robot, home, obj.robot.BodyNames{i}, obj.robot.BodyNames{i-1});
                R = round(R(1:3,1:3));
                tf = obj.transf(R, obj.d(i,:)') * obj.rotZ(obj.q(i));
                T = T * tf;
                Ti{i} = T;
            end
            Tci = arrayfun(@(x) SX.zeros(4,4), 1:obj.N, 'UniformOutput', 0); % arrayfun(@(x) sym(zeros(4, 4)), 1:N, 'UniformOutput', 0);
            for i = 1:obj.N
                Tci{i} = Ti{i} * obj.transl(obj.CoM(i,:)');
            end
        end
        function f = CreateFunction(obj, T)
            % Create a function from the input Symbolic object
            P = [reshape(obj.d,3*obj.N,1); obj.m; reshape(obj.CoM,3*obj.N,1)];
            f = casadi.Function('f', {obj.q_act, P}, {T}, {'q', 'p'}, {'t'});
        end
        function [Jv, Jw] = Jacobians(obj, Ti, Tci)
            % angular velocity Jacobian Jw (1 by N) cell, each cell is a (3 by N)
            % symbolic matrix
            import casadi.*

            Jw = arrayfun(@(x) SX.zeros(3,obj.N), 1:obj.N, 'UniformOutput', 0); % arrayfun(@(x) sym(zeros(3, N)), 1:N, 'UniformOutput', 0);
            for link = 1:obj.N
                for i = 1:link
                    R_i = Ti{i}(1:3,1:3);
                    Jw{link}(:, i) = R_i*[0;0;1]; % sym(R_i*[0;0;1]);
                end
            end
            
            % velocity Jacobian Jv (1 by n) cell, each cell is a (3 by n) symbolic matrix
            Jv = arrayfun(@(x) SX.zeros(3,obj.N), 1:obj.N, 'UniformOutput', 0); % arrayfun(@(x) sym(zeros(3, N)), 1:N, 'UniformOutput', 0);
            for link = 1:obj.N
                j = 1; % actuated joint index
                on = Tci{link}(1:3, 4); % end effector position
                for i = 1:link
                    if obj.robot.Bodies{i}.Joint.Type ~= "fixed"
                        j = i;
                    end
                    oi_1 = Ti{j}(1:3, 4); % joint position
                    Jv{link}(:, i) = cross(Jw{obj.N}(:, i), on - oi_1);
                end
            end
        end
        function Ja = AnalyticJacobian(obj, jv, jw)
            % Calculates the analytical Jacobian (6 by N). It is a function
            % of q.
            import casadi.*
            
            J = [jv; jw];
            Ja = [SX.eye(3) SX.zeros(3,3); SX.zeros(3,3) inv(obj.Bmatrix(obj.alpha))]*J;
        end
        function X_ddot = TaskspaceAcc(obj, Ja)
            % Calculates accelerations in the task space. Returns a 6 by 1
            % symbolic function containing translational and rotational accelerations
            import casadi.*

            qdd = SX.sym('qdd', obj.N-obj.N_fixed, 1);
            Ja_dot = SX.sym('Ja_dot', 6, obj.N-obj.N_fixed);
            for i = 1:6 % iterate all element
                for j = 1:obj.N-obj.N_fixed
                    for k = 1:obj.N-obj.N_fixed
                        Ja_dot(i, j) = jacobian(Ja(i, j), obj.q_act(k))*obj.qd_act(k);
                    end
                end
            end
            x_ddot = Ja(:,1:obj.N-obj.N_fixed) * qdd + Ja_dot * obj.qd_act;
            States = [obj.q_act; obj.qd_act; qdd];
            P = [reshape(obj.d,3*obj.N,1); obj.m; reshape(obj.CoM,3*obj.N,1)];
            X_ddot = Function('X_ddot', {States,   obj.alpha,    P},   {x_ddot}, ...
                                        {'states', 'alpha',     'p'}, {'x_ddot'});
        end
    end
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
            %Transformation for R, d in 3D
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
        function B = Bmatrix(alpha)
            %B matrix that transfer geometric Jacobian to Analytic Jacobian
            phi = alpha(1);
            theta = alpha(2);
            psi = alpha(3);
            B = [cos(psi)*sin(theta) -sin(psi) 0;
                 sin(psi)*sin(theta)  cos(psi) 0;
                 cos(theta)           0        1];
        end
    end
end