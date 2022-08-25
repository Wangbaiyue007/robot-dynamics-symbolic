function [patch_obj, verts] = linkGeneration(color)
    arguments
        color.face (1,1) string = '#4DBEEE'
        color.edge (1,1) string = 'none'
    end
    % link dimensions
    arm_len = 1.0;
    arm_wid = 0.1;
    arm_hei = 0.1;

    % number of vertices of a pi arc
    arc_res = 22 

    %Containters to store coordinates
    x_cl = [];
    y_cl = [];
    z_cl = [];

    % HOW POINTS ARE GENERATED:

    %   left half   no-points    right half
    %      * *                   * *
    %    *                           *
    %   *                             *
    %   *                             *
    %    *                           *
    %      * *                   * *


    % create the left half of the circle
    increments = linspace(pi/2, pi*3/2, arc_res);
    for i=1:arc_res
        x = cos(increments(i))*arm_wid*0.5;
        y = sin(increments(i))*arm_wid*0.5;
        x_cl = [x_cl, x];
        y_cl = [y_cl, y];
    end

    % create the right half of the circle
    increments = linspace(pi/2, -pi/2, arc_res);
    for i=1:arc_res
        x = cos(increments(i))*arm_wid*0.5 + arm_len; % apply the offset in x
        y = sin(increments(i))*arm_wid*0.5;
        x_cl = [x_cl, x];
        y_cl = [y_cl, y];
    end

    % Z coordinates calculations: simple offset from XoY Plane
    z_cl1 = ones(1, 2*arc_res)*arm_hei*0.5; % Offset of points in the positive and neg dir
    z_cl2 = ones(1, 2*arc_res)*arm_hei*-0.5;
    z_cl = [z_cl1, z_cl2]; % combining into one vector of z coordinates

    % Extend the size of points in X, Y to cover both positive and negative Z offset size
    x_cl = [x_cl, x_cl];
    y_cl = [y_cl, y_cl];

    ax = gca;
    tris = convhull(x_cl,y_cl,z_cl); % Generate mesh: the connections between the vertices
    verts = [x_cl', y_cl', z_cl'];
    patch_obj= patch('Faces',tris,'Vertices', verts); % create the object to display
    patch_obj.FaceColor = color.face; %ax.ColorOrder(2,:);
    patch_obj.EdgeColor = color.edge;
    grid on

    %xlim([-2 2])
    %ylim([-2 2])
    %zlim([-1 1])

    %camlight
    %axis equal
    %az = 45;
    %el = 60;
    %view(az, el);
    %pause(33)
