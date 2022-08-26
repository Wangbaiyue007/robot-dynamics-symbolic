N = 7;
d_data = zeros(N, 3);
m_data = zeros(N, 1);
CoM_data = zeros(N, 3);
I_data = zeros(N, 6);
for i = 1:N
    m_data(i) = robot.Bodies{i}.Mass;
    CoM_data(i,:) = robot.Bodies{i}.CenterOfMass;
    I_data(i,:) = robot.Bodies{i}.Inertia;
end
T = robot.getTransform(homeConfiguration(robot), ...
        robot.BodyNames{1}, robot.BaseName);
d_data(1, :) = T(1:3, 4)';
for i = 2:N
    T = robot.getTransform(homeConfiguration(robot), ...
         robot.BodyNames{i}, robot.BodyNames{i-1});
    d_data(i, :) = T(1:3, 4)';
end