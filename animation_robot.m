%% This file is just a visualization of simulation. 
% No simulation process is calculated here. But you can adjust visual
% settings.
close all
clc

figure

% quivers = quiver3([0.0; 0.0], [0.0; 0.0], [0.0; 0.0], [0.0; 0.0], [0.0; 0.0], [1.0; 1.0], 1.00, 'Color','b', 'LineWidth', 2.0);

% zero position
[link01, verts01] = linkGeneration(face='none', edge='k');
[link02, verts02] = linkGeneration(face='none', edge='k');
[fr01, pos01, ~, ~, ~] = sphere_fk3(0, 0, 0, 0);
[fr02, pos02, ~, ~, ~] = sphere_fk3(0, 0, 0, l1);
linkTransform(link01, verts01, fr01);
linkTransform(link02, verts02, fr02);

% simulation links
[link1, verts1] = linkGeneration(edge='none');
[link2, verts2] = linkGeneration(edge='none');

%provide an offset for verts 2
z_offset = -0.0;
verts2 = bsxfun(@plus, verts2, [0.0, 0.0, z_offset]);

xlim([-2.25 2.25])
ylim([-2.25 2.25])
zlim([-2 2])

%set(gca,'xtick',[])
set(gca,'xticklabel',[])
%set(gca,'ytick',[])
set(gca,'yticklabel',[])
%set(gca,'ztick',[])
set(gca,'zticklabel',[])

camlight
campos([3 0 0]);
camorbit(20,10,'camera')

axis equal

traj1 = animatedline('Marker','none', 'Color', [0.4, 0.4, 0.4], 'LineWidth', 1.0);
traj2 = animatedline('Marker','none', 'Color', [0.4, 0.4, 0.4], 'LineWidth', 1.0);

%% Data loaded are:
% W: data from the output of ode
th11 = W(:,1)';
th12 = W(:,2)';
th13 = W(:,3)';
th21 = W(:,4)';
th22 = W(:,5)';
th23 = W(:,6)';
th11_dot = W(:,7)';
th12_dot = W(:,8)';
th13_dot = W(:,9)';
th21_dot = W(:,10)';
th22_dot = W(:,11)';
th23_dot = W(:,12)';


for i=1:size(th11,2)
    [fr1, pos1, ~, ~, ~] = sphere_fk3(th11(i),th12(i),th13(i),l1);
    [fr2, pos2, ~, ~, ~] = sphere_fk3(th21(i),th22(i),th23(i),l1,fr1);
    fr2(:,4) = fr1(:,4); % pivot point of link 2
    fr1(:,4) = [0;0;0;1]; % pivot point of link 1

    linkTransform(link1, verts1, fr1)
    linkTransform(link2, verts2, fr2)

    addpoints(traj1, pos1(1), pos1(2), pos1(3));
    addpoints(traj2, pos2(1), pos2(2), pos2(3)+z_offset);
    
%     vec_w1 = sign(th11_dot(i))*0.1 + th11_dot(i);
%     vec_w2 = sign(th21_dot(i))*0.1 + th21_dot(i);
    
    %set(quivers, 'xdata', [0; pos1(1)],'ydata', [0; pos1(2)]);
    %set(quivers, 'xdata', [0; pos1(1)],'ydata', [0; pos1(2)], 'wdata', [vec_w1; vec_w2]);

    axis equal
    xlim([-3 3])
    ylim([-3 3])
    zlim([-3 3])

    drawnow;

end