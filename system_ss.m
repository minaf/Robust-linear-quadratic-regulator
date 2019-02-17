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
    end
end