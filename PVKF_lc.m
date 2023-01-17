function Y = PVKF_lc(x0,P0,z,sigma_h,sigma_v,sigma_velocity,sigma_a,delta_t)
%% PVKF_lc Summary of this function goes here
% A simple implementation of (E)KF in one time step
%% Detailed explanation goes here
    Q = diag([zeros(3,1);1/delta_t*sigma_a^2*ones(3,1)]);
    R = diag([sigma_h^2;sigma_h^2;sigma_v^2;sigma_velocity^2*ones(3,1)]);
    Phi = eye(6);
    Phi(1,4) = delta_t;
    Phi(2,5) = delta_t;
    Phi(3,6) = delta_t;
    Gamma = diag([zeros(3,1);delta_t*ones(3,1)]);
    H = eye(6);
    x = x0;
    P = P0;
    P = Phi*P*Phi'+Gamma*Q*Gamma';
    K = (H*P*H'+R)\(P*H');
    x = x+K*(z'-H*x);
%     P = (eye(6)-K*H)*P;
    Y = x';
end