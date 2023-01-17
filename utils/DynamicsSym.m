classdef DynamicsSym < KinematicsSym
%Symbolic dynamics class
    properties
        m % mass
        I % inertia vector
        g % gravity
        tau % torque input
        tau_act % torque input after removing fixed joints
        I_tensor % cell of inertia tensors
    end
    
    methods
        function obj = DynamicsSym(robot)
            import casadi.*

            % superclass constructor
            obj@KinematicsSym(robot);

            % new constructor
            obj.m = SX.sym('m', obj.N, 1); % mass
            obj.I = SX.sym('I', obj.N, 6); % inertia vector
            obj.g = SX.sym('g');
            obj.tau = SX.sym('tau', obj.N, 1); % joint torque
            obj.tau_act = obj.tau(obj.act_index);
            % I_tensor (1 by N) cell array, each cell is a (3 by 3) symbolic matrix
            obj.I_tensor = arrayfun(@(joint) obj.InertiaTensor(obj.I(joint, :)), ...
                        1:obj.N, 'UniformOutput', 0);
        end
        function D = MassMatrix(obj, Ti, Jv, Jw)
            import casadi.*

            % D (N by N) symbolic matrix, C (N by N) symbolic matrix
            D = SX.zeros(obj.N, obj.N); % inertia matrix
            for i = 1:obj.N
                R = Ti{i}(1:3,1:3);
                D = D + (obj.m(i)*Jv{i}'*Jv{i} + Jw{i}'*R*obj.I_tensor{i}*R'*Jw{i});
            end
            D = D(obj.act_index, obj.act_index);
        end
        function C = CorioliMatrix(obj, D)
            import casadi.*

            % The Coriolis matrix
            C = SX.zeros(obj.N-obj.N_fixed, obj.N-obj.N_fixed);
            for k = obj.act_index
                for j = obj.act_index
                    for i = obj.act_index
                        c = 1/2 * (jacobian(D(k,j),obj.q(i)) + jacobian(D(k,i),obj.q(j)) - jacobian(D(i,j),obj.q(k)));
                        C(k, j) = C(k, j) + c*obj.qd(i);
                    end
                end
            end
        end
        function G = GravitationMatrix(obj, Tci)
            import casadi.*

            % The gravitation terms
            P = SX.zeros(1); % potential energy
            for i = 1:obj.N
                P = P + obj.m(i) * obj.g * Tci{i}(3, 4);
            end
            G = SX.zeros(obj.N,1);
            for i = obj.act_index
                G(i) = jacobian(P,obj.q(i));
            end
            G = G(obj.act_index);
        end
        function f = ForwardDynamics(obj, D, C, G, useJointConstants)
            arguments
                obj
                D
                C
                G
                useJointConstants logical = 0
            end
            import casadi.*
            
            act_index = obj.act_index;
            N = obj.N;
            if useJointConstants ~= 1
                P = [reshape(obj.d,3*N,1); obj.m; reshape(obj.CoM,3*N,1); reshape(obj.I,6*N,1); obj.g];
                qdd_act = D\(- C * obj.qd_act - G + obj.tau_act);
            else
                friction = SX.sym('Kf', N, 1);
                damping = SX.sym('Kd', N, 1);
                armature = SX.sym('Ka', N, 1);
                P = [reshape(d,3*N,1); obj.m; reshape(obj.CoM,3*N,1); reshape(obj.I,6*N,1); friction; damping; armature; obj.g];
                qdd_act = (D(act_index, act_index) - eye(N).*armature)\((-C(act_index, act_index)+damping) * qd - G(act_index) + friction + obj.tau_act);
            end
            
            % Define input and output
            X = [obj.q_act; obj.qd_act];
            U = obj.tau_act;
            Xdot = [obj.qd_act; qdd_act];
            f = Function('f', { X,   U,   P},  { Xdot }, ...
                              {'x', 'u', 'p'}, {'xdot'});
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

