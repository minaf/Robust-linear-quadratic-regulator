clc; clear; close all;

%define model
A = [1.1 0.5 0;
     0 0.9 0.2;
     0 -0.2 0.8];
 
B = [1 0; 
     0 0.1;
     2 0];
 
[Nx, Nu] = size(B);
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

%define constant from Dean paper 2017
const = 1/(sigma_w*(sqrt(Nx+Nu)+sqrt(Nx)+sqrt(2*log(1/0.95))))^2; 

%% calculate nominal controller
yalmip('clear');
Tf = 1e3; %for calculatinf cost

%decision variables
W = sdpvar(Nx,Nx);    
Z = sdpvar(Nx,Nu,'full');    
X = sdpvar(Nx+Nu,Nx+Nu);    

%using already estimated parameters
Ab = inv_pend_model.A;
Bb = inv_pend_model.B;

%define matrices in the same way as Luo 2005
I = eye(Nx);
H = blkdiag(W, I);
F = [W*Ab'+Z*Bb'; sigma_w*I];
G = -[W Z; zeros(Nx, Nx+Nu)];

T1 = blkdiag(H, W, zeros(Nx+Nu, Nx+Nu));
T1(1:2*Nx, 2*Nx+1:end) = [F G];
T1(2*Nx+1:end, 1:2*Nx) = [F'; G'];

T2 = blkdiag(zeros(2*Nx, 2*Nx), I, -const*(inv_pend_model.D0)); % D = const*(Phi'*Phi)

%define structure
con_str = nominal_controller_structure(T1, T2, W, Z, X);

%calculate controller and its characteristics
controller_nominal = get_nominal_controller(con_str, inv_pend, Tf)

%% calculate dual controller
yalmip('clear');

Tf1 = 20; %for inside of controller optimization

%D is decision variable as well in dual controller case
D = sdpvar(Nx+Nu,Nx+Nu);  
T3 = blkdiag(zeros(2*Nx, 2*Nx), I, -const*(inv_pend_model.D0 + Tf*D)); % D0 = const*(Phi'*Phi)

%define structure for the optimization
con_str = dual_controller_structure(T1, T2, W, Z, X);

%calculate controller and its characteristics
controller_dual = get_dual_controller(con_str, inv_pend, Tf1, Tf);