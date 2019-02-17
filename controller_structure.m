classdef controller_structure
    %state-space form of inverted pendulum
    properties
        T1
        T2
        W
        Z
        X
    end
    methods
        function obj = controller_structure(T1, T2, W, Z, X)
            obj.T1 = T1;
            obj.T2 = T2;
            obj.W = W;
            obj.Z = Z;
            obj.X = X;
        end
        function con = get_nominal_controller(obj, system_real, Tf)
            % designed using initial uncertainty
            [Nx Nu] = get_state_size(system_real);
            tau = sdpvar(1);
            Qcl = system_real.Qcl;
            Rcl = system_real.Rcl;
            
            C = [Qcl*obj.W; Rcl*obj.Z'];
            S = [obj.X C; C' obj.W]; %the cost
            Objective = trace(obj.X);
            Constraints = [obj.T1-tau*obj.T2>=0, obj.W>=0,S>=0, tau>=0];
            ops = sdpsettings('solver','mosek','verbose',0);
            sol = optimize(Constraints,Objective, ops);
            
            K0 = double(obj.Z)'/(double(obj.W));
            cost_old = double(Objective);
            Wnom = double(obj.W);
            cost_new = calculate_cost(K0, system_real, Tf);
            
            tmp = size(obj.T2);
            con = controller_ss(K0, Wnom, cost_old, cost_new, obj.T2(tmp(1)-Nx-Nu+1:end, tmp(2)-Nx-Nu+1:end));    
        end
        
    end
end