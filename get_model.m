function model_app = get_model(obj, N, Ts)

[Nx, Nu] = get_state_size(obj);
A = obj.A;
B = obj.B;
sigma_w = obj.sigma_w;
sigma_u = 1; %covariance of an input

x = cell(N, 1);
u = cell(N, 1);

Phi = zeros(N*(Ts-1), Nx+Nu);
y  = zeros(N*(Ts-1), Nx);

%data collection from the system
for l = 1:N
    x{l}=zeros(Nx, Ts);
    u{l}=sigma_u*randn(Nu, Ts);
    x{1} = zeros(Nx, 1);
    for t=1:Ts-1
        x{l}(:, t+1) = A*x{l}(:,t) + B*u{l}(:,t) + sigma_w*randn(Nx,1); % J: added sigma_w
    end
    Phi((Ts-1)*(l-1) + 1: (Ts-1)*l, :) = [x{l}(:,1:Ts-1)' u{l}(:,1:Ts-1)'];
    y((Ts-1)*(l-1) + 1: (Ts-1)*l, :) = x{l}(:,2:Ts)';
end

%initial parametar estimation
theta = (Phi'*Phi)\Phi'*y;
Ab = theta(1:Nx,:)';
Bb = theta(Nx+1:Nx+Nu,:)';

model_app = model_ss(Ab, Bb, obj.Qcl, obj. Rcl, sigma_w);
model_app.D0 = Phi'*Phi;

end

