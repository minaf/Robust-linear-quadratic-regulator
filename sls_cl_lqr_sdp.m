function res = sls_cl_lqr_sdp(ops)

tstart = tic;

A = ops.A;
B = ops.B;
Q = ops.Q;
R = ops.R;
ea = ops.ea;
eb = ops.eb;
gamma = ops.gamma;

g2 = gamma^2;

[nx,nu] = size(B);

yalmip('clear')

X = sdpvar(nx);
% Z = sdpvar(nu,nx);
Z = sdpvar(nu,nx,'full');
W11 = sdpvar(nx);
W22 = sdpvar(nu);
% W12 = sdpvar(nx,nu);
W12 = sdpvar(nx,nu,'full');
alpha = sdpvar(1);

tmp1 = [X,X,Z';X,W11,W12;Z,W12',W22];

tmp2 = [X - eye(nx),    A*X + B*Z, zeros(nx),        zeros(nx,nu);
        X*A' + Z'*B',   X        , ea*X     ,        eb*Z';
        zeros(nx),      ea*X,      alpha*g2*eye(nx), zeros(nx,nu);
        zeros(nu,nx),   eb*Z,      zeros(nu,nx),     (1-alpha)*g2*eye(nu)];
    
constraints = [tmp1 >= 0, tmp2 >= 0];

obj = (trace(Q*W11) + trace(R*W22))/(1-gamma)^2;


ops = sdpsettings('solver','mosek');
ops.verbose = 0;

sol = solvesdp(constraints,obj,ops);

Z = double(Z);
X = double(X);

K = Z/X;

res.sol = sol;
res.K = K;
res.alpha = double(alpha);
res.obj = double(obj);
res.X = double(X);

res.total_time = toc(tstart);

% Probably better to report inf if infeasible
if sol.problem
    res.obj = inf;
end

yalmip('clear');
