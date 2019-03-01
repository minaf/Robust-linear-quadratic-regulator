classdef dual_controller_structure < controller_structure
%     methods
%         function con = get_dual_controller(obj, system_real, Tf)
%             initial_structure = obj;
%             [Nx, Nu] = get_state_size(system_real);
%             tau = sdpvar(1);
%             Qcl = system_real.Qcl;
%             Rcl = system_real.Rcl;
%             
%             C = [Qcl*obj.W; Rcl*obj.Z'];
%             S = [obj.X C; C' obj.W]; %the cost
%             Objective = trace(obj.X);
%             ops = sdpsettings('solver','mosek','verbose',0);
%             
%             val_t = [0 Inf]; %t and the corresponding minimum objective
%             for t = [linspace(0.005,0.049,10)  linspace(0.05,tau_nom*1.1,10)]
%                 obj = initial_structure;
%                 yalmip('clear')
%                 Constraints = [obj.T1-t*obj.T2>=0, obj.W>=0,S>=0, tau>=0];
%                 sol = optimize(Constraints,Objective, ops);
%                     
%                 if(sol.problem == 0 && double(Objective)<val_t(2))
%                     val_t = [t double(Objective)];
%                     Ztmp = double(Z);
%                     Wtmp = double(W);
%                     tau_tmp = double(tau);
%                     Dtmp = double(D);
%                 end
%             end
%             Kdc = double(Ztmp)'/(double(Wtmp));
%             cost_old = val_t(2);
%             Wdc = double(Wtmp);
%             cost_new = calculate_cost(Kdc, system_real, Tf);
%             
%             tmp = size(obj.T2);
%             con = controller_ss(Kdc, Wdc, cost_old, cost_new, obj.T2(tmp(1)-Nx-Nu+1:end, tmp(2)-Nx-Nu+1:end));
%             
%         end    
%     end
end