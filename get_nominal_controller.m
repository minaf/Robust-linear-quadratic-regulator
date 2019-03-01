function con = get_nominal_controller(obj, model_app, system_real, Tf)
    % designed using initial uncertainty
    assert(Tf>0); %otherwise cost isn't meaningful
    tau = sdpvar(1);
    Qcl = model_app.Qcl;
    Rcl = model_app.Rcl;

    C = [Qcl*obj.W; Rcl*obj.Z'];
    S = [obj.X C; C' obj.W]; %the cost
    Objective = trace(obj.X);
    Constraints = [obj.T1-tau*obj.T2>=0, obj.W>=0,S>=0, tau>=0];
    ops = sdpsettings('solver','mosek','verbose',0);
    sol = optimize(Constraints,Objective, ops);
    
    assert(sol.problem == 0);

    K0 = double(obj.Z)'/(double(obj.W));
    cost_old = double(Objective);
    Wnom = double(obj.W);
    cost_new = calculate_cost(K0, system_real, Tf);

    con = controller_ss(K0, Wnom, cost_old, cost_new);

end