function cost = calculate_cost(Ksim, model, Tf)
    assert(~any(isnan(Ksim(:)))); %otherwise controller doesn't exist
    assert(Tf>0); %otherwise cost isn't meaningful
    A = model.A;
    B = model.B;
    [Nx, Nu] = get_state_size(model);
    W = model.sigma_w*randn(Nx,Tf);
    xs = zeros(Nx,Tf);
    us = zeros(Nu,Tf);
    c = 0;
    for t = 1:Tf
        us(:,t) = Ksim*xs(:,t);
        xs(:,t+1) = A*xs(:,t) + B*us(:,t) + W(:,t);
        c = c + xs(:,t)'*model.Qcl'*model.Qcl*xs(:,t) + us(:,t)'*model.Rcl'*model.Rcl*us(:,t);
    end
    cost = c/Tf;
end