main
%% Test 1 
%function get_model
%check if this is unbiased estimator
%calculate an error and inverse of model uncertainty when using N = 100 experiments
tmp = get_model(inv_pend, 100,5);
tmp_unc1 = norm(tmp.D0, 2);
tmp_cost1 = norm(tmp.A-A) + norm(tmp.B-B);

%calculate an error and inverse of model uncertainty when using N = 1000 experiments
tmp = get_model(inv_pend, 1000,5);
tmp_unc2 = norm(tmp.D0, 2);
tmp_cost2 = norm(tmp.A-A) + norm(tmp.B-B);

%test if the srror is decaying as N is increasing 
assert(tmp_cost1 > tmp_cost2)
%test if norm is decreasing of the model uncertainty as N in increasing
assert(tmp_unc1 < tmp_unc2)

%% Test 2
% check the function spectralRadius
assert(spectralRadius(eye(3)) == 1)
assert(spectralRadius([1 2; 2 1]) == 3)

%% Test 3
%check if the nominal controller is stabilizing the true system
%function: get_nominal_controller(obj, model_app, system_real, Tf)
assert(spectralRadius(A+B*controller_nominal.K) < 1)

%% Test 4
% check if controller is robustly stabilizing (stabilizing all the systems in the confidence set)

%first: check if the worst-case cost is bigger than the cost on true system
assert(controller_nominal.cost_old > controller_nominal.cost_new)

%second: generate some random system 
assert(max_cost_of_confidence_set(inv_pend_model, controller_nominal, const, 1000) < controller_nominal.cost_old)

%% Test 5
% Check function s_dean_lqr_wc which calculates new controller from Dean
% paper
%initial uncertainty
D0 = inv_pend_model.D0;
%calculating spectral norms
M = inv(D0)/const;
ea = sqrt(max(eig(M(1:Nx,1:Nx))));
eb = sqrt(max(eig(M(Nx+1:end,Nx+1:end))));
controller_dean_s = s_dean_lqr_wc(inv_pend_model, ea, eb);
controller_dean = controller_dean_s.K;
 
%check if the controller is stabilizing the true system
assert(spectralRadius(A+B*controller_dean) < 1)

%% Test 6
%Check sls framework sls_cl_lqr_sdp
D0 = inv_pend_model.D0;
M = inv(D0)/const;
ea = sqrt(max(eig(M(1:Nx,1:Nx))));
eb = sqrt(max(eig(M(Nx+1:end,Nx+1:end))));

controller_sls = sls_cl_lqr_sdp(inv_pend_model, ea, eb, 0.2);
%check if the controller is stabilizing the true system
assert(spectralRadius(A+B*controller_sls.K) < 1)







