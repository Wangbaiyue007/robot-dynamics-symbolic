syms th1 th2 th3 th4 th5 th6 d
A4 = [cos(th4) 0 -sin(th4) 0;
    sin(th4) 0 cos(th4) 0;
    0 -1 0 0;
    0 0 0 1];
A5 = [cos(th5) 0 sin(th5) 0;
    sin(th5) 0 -cos(th5) 0;
    0 1 0 0;
    0 0 0 1];
A6 = [cos(th6) -sin(th6) 0 0;
    sin(th6) cos(th6) 0 0;
    0 0 1 d;
    0 0 0 1];

A1 = [cos(th1) 0 -sin(th1) 0;
    sin(th1) 0 cos(th1) 0;
    0 -1 0 0;
    0 0 0 1];
A2 = [cos(th2) 0 sin(th2) 0;
    sin(th2) 0 -cos(th2) 0;
    0 1 0 0;
    0 0 0 1];
A3 = [cos(th3) -sin(th3) 0 0;
    sin(th3) cos(th3) 0 0;
    0 0 1 d;
    0 0 0 1];
T1 = A1*A2*A3;
T2 = T1*A4*A5*A6;