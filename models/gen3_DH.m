function DH = gen3_DH()
%The DH parameters of Kinova Gen3
% DH Parameters: (markdown)
% |    | joint   | parent                 | child                  |        d |   theta |   r |    alpha |
% |---:|:--------|:-----------------------|:-----------------------|---------:|--------:|----:|---------:|
% |  0 | joint_1 | base_link              | shoulder_link          |  0.15643 |       0 |   0 |   0      |
% |  1 | joint_2 | shoulder_link          | half_arm_1_link        |  0.12838 |      -0 |   0 | -89.9996 |
% |  2 | joint_3 | half_arm_1_link        | half_arm_2_link        | -0.01175 |      -0 |   0 | -90.0002 |
% |  3 | joint_4 | half_arm_2_link        | forearm_link           | -0.42076 |       0 |   0 |  90.0002 |
% |  4 | joint_5 | forearm_link           | spherical_wrist_1_link | -0.01275 |      -0 |   0 | -90.0002 |
% |  5 | joint_6 | spherical_wrist_1_link | spherical_wrist_2_link | -0.31436 |       0 |   0 |  90.0002 |
% |  6 | joint_7 | spherical_wrist_2_link | bracelet_link          | -0.00035 |      -0 |   0 | -90.0002 |
% output:
%     r | alpha | d        | theta

DH = [0 pi/2  -(0.1564+0.1284) 0;
      0 pi/2  -(0.0054+0.0064) pi;
      0 pi/2  -(0.2104+0.2104) pi;
      0 pi/2  -(0.0064+0.0064) pi;
      0 pi/2  -(0.2084+0.1059) pi;
      0 pi/2  0                pi;
      0 pi    -(0.1059+0.0615) pi];

end