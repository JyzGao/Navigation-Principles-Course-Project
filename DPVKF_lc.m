function Y = DPVKF_lc(x0,P0,dz,sigma_h,sigma_v,sigma_velocity,sigma_a,delta_t)
%% DPVKF_lc Summary of this function goes here
% A simple implementation of (E)KF in one time step
%% Detailed explanation goes here
    Phi = eye(6);
    Phi(1,4) = delta_t;
    Phi(2,5) = delta_t;
    Phi(3,6) = delta_t;
    H = eye(6);
    Gamma = diag([zeros(3,1);delta_t*ones(3,1)]);
    x = x0;
    P = P0;

    a = 1/delta_t;
    b = a;
    Q = diag([0*sigma_a^2*ones(3,1);a*sigma_a^2*ones(3,1)]);
    R = diag([sigma_h^2+b*sigma_a^2; ...
              sigma_h^2+b*sigma_a^2; ...
              sigma_v^2+b*sigma_a^2; ...
             (sigma_velocity^2+a*sigma_a^2)*ones(3,1)]);
%     x = Phi*x;
    P = Phi*P*Phi'+Gamma*Q*Gamma';
    K = (P*H')*inv(H*P*H'+R);
    x = x+K*(dz'-H*x);
%     P = (eye(6)-K*H)*P;
    Y = x';
end