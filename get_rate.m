function [M_opt, t_opt] = get_rate(A_K, C)
% Get the decay rate of the system
% That is, find lyapunov matrix 'M' and rate 't' s.t.
% M >= C^T C
% V(x) = x^T M x
% d/dt V(x) <= -t*V(x)

M = sdpvar(12,12, 'symmetric');
F = [M >= C'*C];
F = [F, A_K'*M + M*A_K <= -2*.01*M];
optimize(F, [], sdpsettings('solver', 'mosek'));

M_opt = value(M);

end

