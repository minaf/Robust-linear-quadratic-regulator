function res = stochLQRcost(model,nominal_controller)

A = model.A;
B = model.B;
Q = model.Qcl'*model.Qcl;
R = model.Rcl'*model.Rcl;
W = model.sigma_w^2*eye(size(A, 1));
K = nominal_controller.K;


ABK = A+B*K;

X = dlyap(ABK',Q + K'*R*K);

obj = trace(X*W); % naive encoding (should be fine for low dimension)

res = double(obj);

end