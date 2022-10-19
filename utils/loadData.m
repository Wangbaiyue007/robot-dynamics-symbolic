function [d_data, m_data, CoM_data, I_data, fda_data] = loadData(robot)
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

    % joint constants
    fda_data = [
    0.5217383101288284, 10.5, 8.039;
    0.5769579059927288, 7.406484581723072, 11.996202461530364;
    0.4213397946418778, 9.972763340817286, 9.002542786175152;
    0.4945515376566732, 8.266795082250392, 11.580643931670636;
    0.16110705026613545, 8.857224902652815, 8.466504091791412
    0.13334911855141302, 8.711083156933284, 8.853706937374243;
    0.143444018171737, 8.888190363830693, 8.858730366468532
    ];
    fda_data = fda_data(1:N, :);

end