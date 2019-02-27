function res = max_cost_of_confidence_set(model, nominal_controller, N)

%sample some systems
costs = zeros(N,1);
D0 = model.D0;
Ab0 = model.A;
Bb0 = model.B;

for i = 1:N
    
    dA = randn(Nx,Nx);
    dB = randn(Nx,Nu);

    X = [dA';dB'];
    
    tmp = X'*(const*D0)*X;
    
    while max(eig(tmp)) >= 1
        X = 0.95*X;
        tmp = X'*(const*D0)*X;
    end
        
    mod.A = Ab0 + X(1:Nx,:)';
    mod.B = Bb0 + X(Nx+1:end,:)';

    cost = stochLQRcost(model, nominal_controller);
    costs(i) = cost.cost;
    
end
res = max(costs);

