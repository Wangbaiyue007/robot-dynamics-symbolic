% ********** File: planar_fk2.m ********** 
% by Jakub Tomasz Kaminski

function [fr1, pos1, fr2, pos2] = planar_fk2(th1, th2)
    % PLEASE NOTE: this function returns orientation of each link
    %              and position of its **pivot point**
    %              as well as link end positions
    % Convert deg to rad
    th1 = th1 * (pi /180);
    th2 = th2 * (pi /180);

    % DH Table values 
    a1 = 1; alpha1 = 0; d1 = 0; theta1 = th1;
    a2 = 1; alpha2 = 0; d2 = 0; theta2 = th2; 

    % Compose a matrix out of the DH values provided
    DH = [a1 alpha1 d1 theta1;
          a2 alpha2 d2 theta2];

    A1 = HomogenousTransf(DH(1,:));
    A2 = HomogenousTransf(DH(2,:));
     
    fr1 = A1;
    fr1(:,4) = [0;0;0;1]; % pivot point of link 1

    T01 = A1; % end of link 1 with respect to the base frame
    T02 = A1*A2; % end effector with respect to the base frame
    fr2 = T02;
    fr2(:,4) = T01(:,4); % pivot point of link 2
    
    pos1 = T01(1:3,4); % end of link 1 position
    pos2 = T02(1:3,4); % end effector position
end