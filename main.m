clc; clear; close all;

%define model
A = [1.1 0.5 0;
     0 0.9 0.2;
     0 -0.2 0.8];
 
B = [1 0; 
     0 0.1;
     2 0];
 
%define measure matrices
Q = eye(3);
R = diag([1 10]);

%calculate cholesky factors (Q = Qcl'*Qcl)
Qcl = chol(Q);
Rcl = chol(R);

%covariance of a disturbance
sigma_w = 0.5;

%define an instance: inverted pendulum
inv_pend = system_ss(A, B, Qcl, Rcl, sigma_w);

N = 100; % number of experiments
Ts = 5; %number of rollouts

%approximate the system
inv_pend_model = get_model(inv_pend, N, Ts);

