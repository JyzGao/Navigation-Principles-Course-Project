function Y = DPVKF(x0,P0,u,z,sigma_h,sigma_v, ...
                    sigma_velocity,sigma_a,delta_t,delta_t_gps,t_end)
%% DPVKF Summary of this function goes here
% A simple implementation of (E)KF
%% Detailed explanation goes here
    t = 0;
    Q = diag([zeros(3,1);t/delta_t*sigma_a^2*ones(3,1)]);
    Phi = eye(6);
    Phi(1,4) = delta_t;
    Phi(2,5) = delta_t;
    Phi(3,6) = delta_t;
    H = eye(6);
    Gamma = diag([zeros(3,1);delta_t*ones(3,1)]);
    x = x0;
    P = P0;
    Y = u(1,:);
    dz = z;
    for i = 1:length(dz)
        dz(i,:) = u(1+(i-1)*round(delta_t_gps/delta_t),:)-dz(i,:);
    end
    i = 0;
    
    while(t<t_end-delta_t_gps)
        i = i+1;
%         t = t + delta_t_gps;
        for t=t:delta_t:t+delta_t_gps-delta_t
            x = Phi*x;
        end
        t = t + delta_t;
        a = t/delta_t;
        R = diag([sigma_h^2+a*sigma_a^2; ...
                  sigma_h^2+a*sigma_a^2; ...
                  sigma_v^2+a*sigma_a^2; ...
                 (sigma_velocity^2+a*sigma_a^2)*ones(3,1)]);
        x = Phi*x;
        P = Phi*P*Phi'+Gamma*Q*Gamma';
        K = (H*P*H'+R)\(P*H');
        x = x+K*(dz(1+i,:)'-H*x);
        P = (eye(6)-K*H)*P;
        Y = [Y;u(1+(t/delta_t),:)-x'];
    end
end