classdef system_ss
    %state-space form of inverted pendulum
    properties
        A % state-transition matrix
        B % input matrix
        Qcl %factors of measure matrices
        Rcl
        sigma_w = 0.5 %covariance of a disturbance
    end
    methods
        function obj = system_ss(A, B, Qcl, Rcl, sigma_w)
            obj.A = A;
            obj.B = B;
            obj.Qcl = Qcl;
            obj.Rcl = Rcl;
            obj.sigma_w = sigma_w;
        end
        function [Nx, Nu] = get_state_size(obj)
            Nx = size(obj.A,1);
            Nu = size(obj.B,2);
        end
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
    end
end