function res = s_dean_lqr_wc(model, ea, eb)

% Then fit a model
% ops_sls.A = A_ls;
% ops_sls.B = B_ls;
% ops_sls.Q = Q;
% ops_sls.R = R;
% ops_sls.ea = res_bs.ea;
% ops_sls.eb = res_bs.eb;

[nx,nu] = size(model.B);

tstart = tic;

% Simple golden-section search for gamma
gr = (1 + sqrt(5))/2;
gsa = 0.5;
gsb = 1;
gsc = gsb - (gsb - gsa)/gr;
gsd = gsa + (gsb - gsa)/gr;

max_gs_iters = 10;
gs_iter = 1;

while (abs(gsc-gsd) > 0.001) && (gs_iter <= max_gs_iters)
   
% Compute at c  
    gamma = gsc;
    res_sls_c = sls_cl_lqr_sdp(model, ea, eb, gamma);
    fc = res_sls_c.obj;
    
% Compute at d 
    gamma = gsd;
    res_sls_d = sls_cl_lqr_sdp(model, ea, eb, gamma);
    fd = res_sls_d.obj;   
    
   % fprintf('\tGS itr: %d, prob: %d, a: %.3f, b: %.3f, c: %.3f, d: %.3f, |gc-gd| = %.4f, Obj(c) = %.5e, Obj(d) = %.5e\n',gs_iter,res_sls_c.sol.problem,gsa,gsb,gsc,gsd,abs(gsc-gsd),fc,fd)

 
    if fc < fd
        gsb = gsd;
    else
        gsa = gsc;
    end
    
    gsc = gsb - (gsb - gsa)/gr;
    gsd = gsa + (gsb - gsa)/gr;
    
    gs_iter = gs_iter + 1;
        
end

comptime_wc= toc(tstart);

% Note: changing as 17/5/18

fprintf('Checking feasibility:\n')

% Check for feasability
if ~res_sls_d.sol.problem
    fprintf('\td feasible\n')
    obj_d = res_sls_d.obj;
    feas_d = true;
else
    fprintf('\td INfeasible\n')
    obj_d = inf; % for comparison
    feas_d = false;
end
    
if ~res_sls_c.sol.problem
    fprintf('\tc feasible\n')
    obj_c = res_sls_c.obj;
    feas_c = true;
else
    fprintf('\tc INfeasible\n')
    obj_c = inf; % for comparison
    feas_c = false;
end
    
    
fprintf('Checking objectives:\n')
if feas_c || feas_d
    
    if obj_d < obj_c
        fprintf('\tUsing d result\n')
        K_sls = res_sls_d.K;
    else
        fprintf('\tUsing c result\n')
        K_sls = res_sls_c.K;
    end
    
%     res_cost_sls = stochLQRcost(A,B,K_sls,Q,R,W); % note: compute cost on true system
%     cost_sls = res_cost_sls.cost;
    
    rbst_wc_feas = true;
else
    K_sls = nan(nu,nx);
%     cost_sls = nan;
    rbst_wc_feas = false;
end


%fprintf('Feasible: %d\n',rbst_wc_feas)


%%

res.K = K_sls;
res.feasibility = rbst_wc_feas;
res.time = comptime_wc;
res.gs_iters = gs_iter-1;


res.res_sls_c = res_sls_c;
res.res_sls_d = res_sls_d;





end