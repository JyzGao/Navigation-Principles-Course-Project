clc
clear
close all


%% Parameter Settings
f_imu = 200;              % Sampling frequency of IMU, unit: Hz.
f_gps = 1;               % Sampling frequency of GPS, unit: Hz.


%% Actual Object Path
% "L" Path, using ENU coordinate
%     N          X
%     ↑          ↑
%  W<-|->E       |
%     ↓          |
%     S          |  200m
%                |
%                |
%        100m    |
%    X---------->X
%
%
h = 1/f_imu;
load("L_Path.mat");


%% Simulated Object Path
sigma_a = sqrt(0.01);        % Standard deviation of accelaration error
sigma_h = 1.6e0;            % Standard deviation of horizontal error
sigma_v = 3.0e0;            % Standard deviation of horizontal error
sigma_velocity = 0.1;       % Standard deviation of velocity

ax_imu1 = ax;
ay_imu1 = ay;
az_imu1 = az;
vx_imu1 = vx;
vy_imu1 = vy;
vz_imu1 = vz;
x_imu1 = x;
y_imu1 = y;
h_imu1 = h;
ax_imu2 = ax;
ay_imu2 = ay;
az_imu2 = az;
vx_imu2 = vx;
vy_imu2 = vy;
vz_imu2 = vz;
x_imu2 = x;
y_imu2 = y;
h_imu2 = h;
vx_gps = [vx(1)];
vy_gps = [vy(1)];
vz_gps = [vz(1)];
x_gps = [x(1)];
y_gps = [y(1)];
h_gps = [h(1)];


%% Loosly-coupled KF for Position and Velocity Estimation
for i = 2:length(t)
    %% IMU
    ax_imu1(i) = ax_imu1(i)+normrnd(0,sigma_a);
    ay_imu1(i) = ay_imu1(i)+normrnd(0,sigma_a);
    az_imu1(i) = az_imu1(i)+normrnd(0,sigma_a);
    vx_imu1(i) = vx_imu1(i-1)+ax_imu1(i-1)/f_imu;
    vy_imu1(i) = vy_imu1(i-1)+ay_imu1(i-1)/f_imu;
    vz_imu1(i) = vz_imu1(i-1)+az_imu1(i-1)/f_imu;
    x_imu1(i) = x_imu1(i-1)+vx_imu1(i-1)/f_imu;
    y_imu1(i) = y_imu1(i-1)+vy_imu1(i-1)/f_imu;
    h_imu1(i) = h_imu1(i-1)+vz_imu1(i-1)/f_imu;

    ax_imu2(i) = ax_imu2(i)+normrnd(0,sigma_a);
    ay_imu2(i) = ay_imu2(i)+normrnd(0,sigma_a);
    az_imu2(i) = az_imu2(i)+normrnd(0,sigma_a);
    vx_imu2(i) = vx_imu2(i-1)+ax_imu2(i-1)/f_imu;
    vy_imu2(i) = vy_imu2(i-1)+ay_imu2(i-1)/f_imu;
    vz_imu2(i) = vz_imu2(i-1)+az_imu2(i-1)/f_imu;
    x_imu2(i) = x_imu2(i-1)+vx_imu2(i-1)/f_imu;
    y_imu2(i) = y_imu2(i-1)+vy_imu2(i-1)/f_imu;
    h_imu2(i) = h_imu2(i-1)+vz_imu2(i-1)/f_imu;

    %% GPS
    if(mod(i-1,round(f_imu/f_gps))==0)

        vx_gps = [vx_gps;vx(i)+normrnd(0,sigma_velocity)];
        vy_gps = [vy_gps;vy(i)+normrnd(0,sigma_velocity)];
        vz_gps = [vz_gps;vz(i)+normrnd(0,sigma_velocity)];
        sample = mvnrnd([0 0],[sigma_h 0;0 sigma_h],1);
        x_gps = [x_gps;x(i)+sample(1)];
        y_gps = [y_gps;y(i)+sample(2)];
        h_gps = [h_gps;h(i)+normrnd(0,sigma_v)];

        z_gps = [x_gps(end),y_gps(end),h_gps(end),...
                 vx_gps(end),vy_gps(end),vz_gps(end)];
        z_imu1 = [x_imu1(i),y_imu1(i),h_imu1(i),vx_imu1(i),vy_imu1(i),vz_imu1(i)];
        z_imu2 = [x_imu2(i),y_imu2(i),h_imu2(i),vx_imu2(i),vy_imu2(i),vz_imu2(i)];
        dz = z_imu2-z_gps;
%         x0 = [0;0;10;0;0;0];
        P0 = diag(1e0*ones(6,1));
        Y1 = PVKF_lc(z_imu1',P0,z_gps,sigma_h,sigma_v, ...
            sigma_velocity,sigma_a,1/f_imu);
        x_imu1(i) = Y1(1);
        y_imu1(i) = Y1(2);
        h_imu1(i) = Y1(3);
        vx_imu1(i) = Y1(4);
        vy_imu1(i) = Y1(5);
        vz_imu1(i) = Y1(6);

        Y2 = DPVKF_lc(zeros(6,1),P0,dz',sigma_h,sigma_v, ...
            sigma_velocity,sigma_a,1/f_imu);
        x_imu2(i) = x_imu2(i)-Y2(1);
        y_imu2(i) = y_imu2(i)-Y2(2);
        h_imu2(i) = h_imu2(i)-Y2(3);
        vx_imu2(i) = vx_imu2(i)-Y2(4);
        vy_imu2(i) = vy_imu2(i)-Y2(5);
        vz_imu2(i) = vz_imu2(i)-Y2(6);
    end
end


%% Plot
plot3(x,y,h,'k',"LineWidth",1.5);
hold on;
plot3(x_gps,y_gps,h_gps,'o',"Color",[1 0.5 0]);
xlabel('East');
ylabel('North');
zlabel('Height');
axis([0 105 -5 205 0 20]);
axis equal;
grid minor;
hold on;
plot3(x_imu1,y_imu1,h_imu1,'r',"LineWidth",2);
hold on;
plot3(x_imu2,y_imu2,h_imu2,'b',"LineWidth",2);
legend('Actual Path','GPS Measurement', ...
    'Loosly-coupled KF Estimation','Loosly-coupled DKF Estimation');

