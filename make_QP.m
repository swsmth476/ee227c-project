function [H, f, A_constr, b_constr] = make_QP(T, uT, uK, A, B, theta, x0, Qf, qf, Q, q, R, r, Hx, hx, Hu, hu)
    
    %%% augmented system form %%%
    
    % system is assumed to be in augmented form %
    % augmented state is [x(t) u(t)]^T %
    % du(t) is input to augmented system %
    
    % input to system updates as %
    % u(t+1) = u(t) + du(t) %
    
    % dynamics are %
    % [x(t)] = [A B][x(t)] + [0][du(t)] + [theta]
    % [u(t)]   [0 I][u(t)]   [I]          [  0  ]
    
    %%% move blocking (reduces complexity) %%%
    
    % full time horizon = T %
    % control time horizon = uT %
    % move blocking factor = uK %
    
    % only change inputs every uK steps %
    % set input constant value after uT steps %
    
    %%% make cost function for MPC %%%
    
    % ( note this cost is defined for the augmented system (see above) %
    %  with state [x(t) u(t)]^T and input du(t) ) %
    
    % goal is to minimize %
    % 1/2 x(t+T)^T*Qf*x(t+T) + x(t+T) + \sum_{k = t}^{t+N) J(k)
    
    % J(t) = 1/2*x(t)^T*Q*x(t) + x(t)^T*q
    %       + 1/2*u(t)^T*R*u(t) + u(t)^T*r (1)
    
    % the following code produces an H and f s.t.
    % min 1/2*u^T*H*u + f^T*u
    % where u = [u(t) u(t+1) u(t+2) ... u(t+T-1)]'
    % is equivalent to (1) above
    
    %%% make constraints for MPC %%%
    
    % goal is to create Au <= b constraints s.t.
    
    % 1. Hx*x(k) <= hx for t <= k <= t+T
    % 2. Hu*u(k) <= hu for t <= k <= t+T

    n = size(x0,1); % state dimension
    m = size(B,2); % input dimension
    
    AB = zeros(n, T*m);
    M = zeros(n, T);
    
    AB(:,1:m) = B;
    M(:,1) = A*x0 + theta;
    
    % store all A^k*B and M(k)
    for i = 1:T-1
        index1 = (i*m+1):(i*m+m);
        index2 = index1 - m;
        AB(:,index1) = A*AB(:,index2);
        M(:,i+1) = A*M(:,i) + theta;
    end
    M = reshape(M, n*T, 1);
    
    G = zeros(n*T, m*T);
    
    for i = 0:T-1
        rows = (i*n+1):(i*n + n);
        cols = 1:m;
        while(rows(1) < n*T)
            G(rows, cols) = AB(:,(i*m+1):(i*m+m));
            rows = rows + n;
            cols = cols + m;
        end
    end
    
    Q_bar = [];
    q_bar = [];
    R_bar = [];
    r_bar = [];
    Hx_bar = [];
    hx_bar = [];
    Hu_bar = [];
    hu_bar = [];
    
    for i = 1:T-1
        Q_bar = blkdiag(Q_bar, Q);
        R_bar = blkdiag(R_bar, R);
        Hx_bar = blkdiag(Hx_bar, Hx);
        Hu_bar = blkdiag(Hu_bar, Hu);
        q_bar = [q_bar; q];
        r_bar = [r_bar; r];
        hx_bar = [hx_bar; hx];
        hu_bar = [hu_bar; hu];
    end
    
    Hx_bar = blkdiag(Hx_bar, Hx);
    Hu_bar = blkdiag(Hu_bar, Hu);
    hx_bar = [hx_bar; hx];
    hu_bar = [hu_bar; hu];
    R_bar = blkdiag(R_bar, R);
    r_bar = [r_bar; r];
    
    % add final state costs %
    Q_bar = blkdiag(Q_bar, Qf);
    q_bar = [q_bar; qf];
    
    % construct QP weights %
    H = G'*Q_bar*G + R_bar;
    f = G'*Q_bar*M + G'*q_bar + r_bar;
    
    % add QP constraints %
    A_constr = Hx_bar*G;
    b_constr = hx_bar - Hx_bar*M;
    
    A_constr = [A_constr; Hu_bar];
    b_constr = [b_constr; hu_bar];
    
    % transform problem to include move blocking %
    num_moves = ceil(uT/uK);
    
    % construct transformation matrix T %
    T = zeros(T*m, num_moves*m);
    
    for i = 0:num_moves-1
        rows = (i*m*uK+1):(i*m*uK+m);
        cols = (i*m+1):(i*m+m);
        T(rows, cols) = eye(m);
    end
    
    % compute transformations %
    H = T'*H*T;
    f = f'*T;
    A_constr = A_constr*T;

end