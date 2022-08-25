function qdd = ForwardDynamicsSym(D, C, G, q, qd, tau)
%Calculate forward dynamics from symbolic input
% input: D, C, G symbolic matrices, fs joint parameters, q, qd joint states
N = size(D, 1);
q_sym = arrayfun(@(n) sym(append('q', num2str(n)), 'real'), 1:N)';
qd_sym = arrayfun(@(n) sym(append('qd', num2str(n)), 'real'), 1:N)';


D = subs(D, q_sym, q);
C = subs(C, [q_sym qd_sym], [q qd]);
G = subs(G, q_sym, q);

Dval = double(D);
Cval = double(C);
Gval = double(G);

qdd = (Dval) \ (- Cval * qd - Gval + tau);

end

