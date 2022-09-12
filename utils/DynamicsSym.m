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
        fs_sym % joint parameters
        g % gravity
        tau % torque input
        DataFormat % row or column
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
    end
end

