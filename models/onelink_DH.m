function DH = onelink_DH()
%1LINK_DH DH parameters of 1link
% DH Parameters: (markdown)
% |    | joint   | parent    | child           |        d |   theta |   r |   alpha |
% |---:|:--------|:----------|:----------------|---------:|--------:|----:|--------:|
% |  0 | joint   | base_link | half_arm_1_link | -0.12838 |       0 |   0 | 90.0002 |
% output:
% r | alpha | d | theta
DH = [0 pi -0.12838 0];

end

