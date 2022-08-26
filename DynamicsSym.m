classdef DynamicsSym
%Symbolic dynamics class
    properties
        N % dimension
        q_sym % general coordinates
        qd_sym % general velocities
        d_sym % symbolic translation 
        m_sym % mass
        CoM_sym % center of mass offset
        I_sym % inertia vector
        g % gravity
    end
    
    methods
        function obj = DynamicsSym(robot)
            obj.N = robot.NumBodies;
            N = obj.N;
            obj.q_sym = sym('q', [N 1], 'real');
            obj.qd_sym = sym('qd', [N 1], 'real');
            obj.d_sym = sym('d', [N 3], 'real');
            obj.m_sym = sym('m', [N 1], 'real');
            obj.CoM_sym = sym('c', [N 3], 'real');
            obj.I_sym = sym('I', [N 6], 'real');
            obj.g = sym('g', 'real');
        end
        function [qdd, Dval, Cval, Gval] = ForwardDynamics(obj, robot, D, C, G, q, qd, tau)
            %Calculate forward dynamics from symbolic input
            % input: D, C, G symbolic matrices, fs joint parameters, q, qd joint states
            d = zeros(obj.N, 3);
            m = zeros(obj.N, 1);
            CoM = zeros(obj.N, 3);
            I = zeros(obj.N, 6);
            for i = 1:obj.N
                m(i) = robot.Bodies{i}.Mass;
                CoM(i,:) = robot.Bodies{i}.CenterOfMass;
                I(i,:) = robot.Bodies{i}.Inertia;
            end
            T = robot.getTransform(homeConfiguration(robot), ...
                    robot.BodyNames{1}, robot.BaseName);
            d(1, :) = T(1:3, 4)';
            for i = 2:obj.N
                T = robot.getTransform(homeConfiguration(robot), ...
                    robot.BodyNames{i}, robot.BodyNames{i-1});
                d(i, :) = T(1:3, 4)';
            end

            G = subs(G, obj.g, 9.8);
            D = subs(D, [obj.q_sym, obj.d_sym, obj.m_sym, obj.CoM_sym, obj.I_sym], [q, d, m, CoM, I]);
            C = subs(C, [obj.q_sym, obj.qd_sym, obj.d_sym, obj.m_sym, obj.CoM_sym, obj.I_sym], [q, qd, d, m, CoM, I]);
            G = subs(G, [obj.q_sym, obj.d_sym, obj.m_sym, obj.CoM_sym, obj.I_sym], [q, d, m, CoM, I]);
            
            Dval = double(D);
            Cval = double(C);
            Gval = double(G);
            
            qdd = (Dval) \ (- Cval * qd - Gval + tau);
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
    end
end

