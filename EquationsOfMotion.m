function [Dval, Cval] = EquationsOfMotion(m,r,l)
%% Built-in physics, no need to worry
% Measurements of the 2-link arm
m1=m; r1=r; l1=l;
g=9.8;

%% Jacobians
syms th11 th12 th13 th21 th22 th23
[T1, pos1, z1, z2, z3] = sphere_fk3(th11,th12,th13,l1);
[T2, pos2, z4, z5, z6] = sphere_fk3(th21,th22,th23,l1,T1);
R1_0 = T1(1:3,1:3);
R2_0 = T2(1:3,1:3);
Jv1 = cross([0;0;1],pos1);
Jv2 = simplify(cross(z1,pos1));
Jv3 = simplify(cross(z2,pos1));
Jvc1 = [Jv1 Jv2 Jv3 zeros(3,3)];
Jv1 = cross([0;0;1],pos2);
Jv2 = simplify(cross(z1,pos2));
Jv3 = simplify(cross(z2,pos2));
Jv4 = simplify(cross(z3,pos2-pos1));
Jv5 = simplify(cross(z4,pos2-pos1));
Jv6 = simplify(cross(z5,pos2-pos1));
Jvc2 = [Jv1 Jv2 Jv3 Jv4 Jv5 Jv6];
Jw1 = [0;0;1];
Jw2 = z1;
Jw3 = simplify(z2);
Jw4 = simplify(z3);
Jw5 = simplify(z4);
Jw6 = simplify(z5);
Jwc1 = [Jw1 Jw2 Jw3 zeros(3,3)];
Jwc2 = [Jw1 Jw2 Jw3 Jw4 Jw5 Jw6];
%% Inertia tensor of each link
I = [1/2*m1*r1^2 0 0;
    0 1/3*m1*l1^2 0;
    0 0 1/3*m1*l1^2];
%% Equations of motion
% n by n inertia matrix
syms Dval(th11,th12,th13,th21,th22,th23)
D = ( simplify(m1.*Jvc1'*Jvc1) + simplify(Jwc1'*R1_0*I*R1_0'*Jwc1) ) + ( (m1.*Jvc2'*Jvc2) + (Jwc2'*R1_0*I*R1_0'*Jwc2) );
Dval = matlabFunction(D);

% Coriolis matrix
syms dkj(th11,th12,th13,th21,th22,th23) dki(th11,th12,th13,th21,th22,th23) dij(th11,th12,th13,th21,th22,th23) ...
    c_sum(th11,th12,th13,th21,th22,th23,q1_d,q2_d,q3_d,q4_d,q5_d,q6_d) Cval(th11,th12,th13,th21,th22,th23) ...
    q1_d q2_d q3_d q4_d q5_d q6_d
q = [th11 th12 th13 th21 th22 th23];
qdot = [q1_d q2_d q3_d q4_d q5_d q6_d];
dkj(th11,th12,th13,th21,th22,th23) = 0;
dki(th11,th12,th13,th21,th22,th23) = 0;
dij(th11,th12,th13,th21,th22,th23) = 0;
c_sum(th11,th12,th13,th21,th22,th23,q1_d,q2_d,q3_d,q4_d,q5_d,q6_d) = 0;
C = sym(zeros(6,6));
for k = 1:6
    for j = 1:6
        for i = 1:6
            dkj(th11,th12,th13,th21,th22,th23) = D(k,j);
            dki(th11,th12,th13,th21,th22,th23) = D(k,i);
            dij(th11,th12,th13,th21,th22,th23) = D(i,j);
            c_sum(th11,th12,th13,th21,th22,th23,q1_d,q2_d,q3_d,q4_d,q5_d,q6_d) = 1/2.*( (diff(dkj,q(i))) + (diff(dki,q(j))) - (diff(dij,q(k))) ) * qdot(i);
            C(k,j) = C(k,j) + c_sum;
        end
    end
end
Cval = matlabFunction(C);

end