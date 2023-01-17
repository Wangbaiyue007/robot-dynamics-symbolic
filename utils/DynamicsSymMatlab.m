classdef DynamicsSymMatlab
%Symbolic dynamics class
    properties
        N % dimension
        q_sym % general coordinates
        qd_sym % general velocities
        d_sym % symbolic translation 
        m_sym % mass
        CoM_sym % center of mass offset
        I_sym % inertia vector
        fs_sym % joint parameters
        g % gravity
        tau % torque input
        DataFormat % row or column
    end
    
    methods
        function obj = DynamicsSymMatlab(robot)
            obj.N = robot.NumBodies;
            N = obj.N;
            obj.q_sym = sym('q', [N 1], 'real');
            obj.qd_sym = sym('qd', [N 1], 'real');
            obj.d_sym = sym('d', [N 3], 'real');
            obj.m_sym = sym('m', [N 1], 'real');
            obj.CoM_sym = sym('c', [N 3], 'real');
            obj.I_sym = sym('I', [N 6], 'real');
            obj.fs_sym = sym('fs', [N 3], 'real');
            obj.g = sym('g', 'real');
            obj.tau = sym('tau', [N 1], 'real');
            obj.DataFormat = convertCharsToStrings(robot.DataFormat);
        end
        function [qdd, Dval, Cval, Gval] = ForwardDynamics(obj, robot, D, C, G, q, qd, tau)
            %Calculate forward dynamics from symbolic input
            %   input: D, C, G symbolic matrices, fs joint parameters, q, qd joint states

            [d, m, CoM, I] = loadData(robot);
            
            G = subs(G, obj.g, 9.8);
            Dval = double(subs(D, [obj.q_sym, obj.d_sym, obj.m_sym, obj.CoM_sym, obj.I_sym], [q, d, m, CoM, I]));
            Cval = double(subs(C, [obj.q_sym, obj.qd_sym, obj.d_sym, obj.m_sym, obj.CoM_sym, obj.I_sym], [q, qd, d, m, CoM, I]));
            Gval = double(subs(G, [obj.q_sym, obj.d_sym, obj.m_sym, obj.CoM_sym, obj.I_sym], [q, d, m, CoM, I]));
            
            qdd = (Dval) \ (- Cval * qd - Gval + tau);
        end
        function SaveFunction(obj, D, C, G, name, usefda)
            % This isn't quite useful becuase it takes too long!
            if usefda
                %dir = -1 + 2./(1 + exp(-100*obj.qd_sym));
                dir = sign(obj.qd_sym);
                if obj.DataFormat == "column"
                    Function = (D + obj.fs_sym(:, 3).*eye(obj.N)) \ ...
                                (-C * obj.qd_sym - G + obj.tau  ...
                                 -obj.fs_sym(:,1) .* dir - obj.fs_sym(:,2) .* obj.qd_sym);
                elseif obj.DataFormat == "row"
                    Function = (-obj.qd_sym' * C' - G' + obj.tau' ...
                                -obj.fs_sym(:,1)' .* dir' - obj.fs_sym(:,2)' .* obj.qd_sym') ...
                                / (D' + obj.fs_sym(:, 3).*eye(obj.N));
                end
                disp("using joint parameters option")
            else
                if obj.DataFormat == "column"
                    Function = D \ (-C * obj.qd_sym - G + obj.tau);
                elseif obj.DataFormat == "row"
                    Function = (-obj.qd_sym' * C' - G' + obj.tau') / D';
                end
            end
            disp(append("robot data format: ", obj.DataFormat));
            matlabFunction(Function, 'File', name);
        end
    end
   
    methods (Static)
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
        function I = parallelAxis(inertia_vec, mass, com)
            %% PARALLEL AXIS
            % Inputs
            %   inertia_vec - the inertia tensor obtained from RigidBody object
            %   mass - mass of the link
            %   com - the center of mass relative to the joint frame
            %
            % Note that RigidBody expresses the inertia tensor with respect to
            % the joint frame. This function uses the parallel-axis theorem to
            % compute the inertia tensor in the link body frame whose origin is
            % located at the link center of mass. This function returns the inertia
            % values that are in the URDF.
        
            % We are computing the following:
            % I_{xx}^{C} = I_{xx}^{A} - m (y_c^2 + z_c^2);
            % I_{yy}^{C} = I_{yy}^{A} - m (x_c^2 + z_c^2);
            % I_{zz}^{C} = I_{zz}^{A} - m (x_c^2 + y_c^2);
            %
            % I_{xy}^{C} = I_{xy}^{A} + m x_c y_c;
            % I_{xz}^{C} = I_{xz}^{A} + m x_c z_c;
            % I_{yz}^{C} = I_{yz}^{A} + m y_c z_c;
            %% compute new inertia tensor
            % mass
            m = mass;
        
            % center of mass coordinates
            x = com(1);
            y = com(2);
            z = com(3);
        
            % inertia vec from matlab rigidBody object
            Ixx = inertia_vec(1);
            Iyy = inertia_vec(2);
            Izz = inertia_vec(3);
            Iyz = inertia_vec(4);
            Ixz = inertia_vec(5);
            Ixy = inertia_vec(6);
        
            % parallel axis theorem (see Craig 6.25)
            Ixx = Ixx + m * (y^2 + z^2);
            Iyy = Iyy + m * (x^2 + z^2);
            Izz = Izz + m * (x^2 + y^2);
        
            Ixy = Ixy - m * x * y;
            Ixz = Ixz - m * x * z;
            Iyz = Iyz - m * y * z;
        
            I = [Ixx, Iyy, Izz, Iyz, Ixz, Ixy];
        end
    end
end

