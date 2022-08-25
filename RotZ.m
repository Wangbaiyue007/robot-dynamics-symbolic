function R = RotZ(q)
%Rotation matrix about z axis for q symbolic angle

R = [cos(q) -sin(q) 0 0;
     sin(q) cos(q)  0 0;
     0      0       1 0;
     0      0       0 1];

end

