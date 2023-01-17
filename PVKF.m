function Y = PVKF(x0,P0,u,z,sigma_h,sigma_v, ...
                    sigma_velocity,sigma_a,delta_t,delta_t_gps,t_end)
%% PVKF Summary of this function goes here
% A simple implementation of (E)KF
%% Detailed explanation goes here
    t = 0;
    i = 0;
%     Q = diag([zeros(3,1);1/delta_t*sigma_a^2*ones(3,1)]);
    R = diag([sigma_h^2;sigma_h^2;sigma_v^2;sigma_velocity^2*ones(3,1)]);
    Phi = eye(6);
    Phi(1,4) = delta_t;
    Phi(2,5) = delta_t;
    Phi(3,6) = delta_t;
    Gamma = diag([zeros(3,1);delta_t*ones(3,1)]);
    H = eye(6);
    x = x0;
    P = P0;
    Y = x0';
    
    while(t<t_end-delta_t_gps)
        i = i+1;
%         t = t + delta_t_gps;
        for t=t:delta_t:t+delta_t_gps-delta_t
            uk = [zeros(3,1);delta_t*u(1+round(t/delta_t),:)'];
            x = Phi*x+uk;
        end
        t = t + delta_t;
        Q = diag([zeros(3,1);t/delta_t*sigma_a^2*ones(3,1)]);
        uk = [zeros(3,1);delta_t*u(1+i*round(delta_t_gps/delta_t),:)'];
        x = Phi*x+uk;
        P = Phi*P*Phi'+Gamma*Q*Gamma';
        K = (P*H')*inv(H*P*H'+R);
        x_dot = x+K*(z(1+i,:)'-H*x);
        P = (eye(6)-K*H)*P;
        Y = [Y;x_dot'];
    end
end