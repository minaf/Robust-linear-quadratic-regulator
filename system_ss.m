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
        function cost = calculate_cost(Ksim, obj, Tf)
            A = obj.A;
            B = obj.B;
            [Nx, Nu] = get_state_size(obj);
            W = obj.sigma_w*randn(Nx,Tf);
            xs = zeros(Nx,Tf);
            us = zeros(Nu,Tf);
            c = 0;
            for t = 1:Tf
                us(:,t) = Ksim*xs(:,t);
                xs(:,t+1) = A*xs(:,t) + B*us(:,t) + W(:,t);
                c = c + xs(:,t)'*obj.Qcl'*obj.Qcl*xs(:,t) + us(:,t)'*obj.Rcl'*obj.Rcl*us(:,t);
            end
            cost = c/Tf;
        end
    end
end