function [d_data, m_data, CoM_data, I_data] = loadData(robot)
arguments
    robot rigidBodyTree
end

    N = robot.NumBodies;
    d_data = zeros(N, 3);
    m_data = zeros(N, 1);
    CoM_data = zeros(N, 3);
    I_data = zeros(N, 6);
    for i = 1:N
        m_data(i) = robot.Bodies{i}.Mass;
        CoM_data(i,:) = robot.Bodies{i}.CenterOfMass;
        % the inertia tensor need to be transformed back to relative to CoM
        I_data(i,:) = parallelAxis(robot.Bodies{i}.Inertia, -m_data(i), -CoM_data(i, :));
    end
    T = robot.getTransform(homeConfiguration(robot), ...
            robot.BodyNames{1}, robot.BaseName);
    d_data(1, :) = T(1:3, 4)';
    for i = 2:N
        T = robot.getTransform(homeConfiguration(robot), ...
             robot.BodyNames{i}, robot.BodyNames{i-1});
        d_data(i, :) = T(1:3, 4)';
    end

end