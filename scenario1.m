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
%% IMU
sigma_a = sqrt(0.01);        % Standard deviation of accelaration error

ax_imu = ax;
ay_imu = ay;
az_imu = az;
vx_imu = vx;
vy_imu = vy;
vz_imu = vz;
x_imu = x;
y_imu = y;
h_imu = h;

for i = 2:length(t)
    ax_imu(i) = ax_imu(i)+normrnd(0,sigma_a);
    ay_imu(i) = ay_imu(i)+normrnd(0,sigma_a);
    az_imu(i) = az_imu(i)+normrnd(0,sigma_a);
    vx_imu(i) = vx_imu(i-1)+ax_imu(i-1)/f_imu;
    vy_imu(i) = vy_imu(i-1)+ay_imu(i-1)/f_imu;
    vz_imu(i) = vz_imu(i-1)+az_imu(i-1)/f_imu;
    x_imu(i) = x_imu(i-1)+vx_imu(i-1)/f_imu;
    y_imu(i) = y_imu(i-1)+vy_imu(i-1)/f_imu;
    h_imu(i) = h_imu(i-1)+vz_imu(i-1)/f_imu;
end
    

%% GPS
sigma_h = 1.6e0;           % Standard deviation of horizontal error
sigma_v = 3.0e0;           % Standard deviation of horizontal error
sigma_velocity = 0.1;       % Standard deviation of velocity

vx_gps = [vx(1)];
vy_gps = [vy(1)];
vz_gps = [vz(1)];
x_gps = [x(1)];
y_gps = [y(1)];
h_gps = [h(1)];

for i = round(f_imu/f_gps):round(f_imu/f_gps):length(t)
    vx_gps = [vx_gps;vx(i)+normrnd(0,sigma_velocity)];
    vy_gps = [vy_gps;vy(i)+normrnd(0,sigma_velocity)];
    vz_gps = [vz_gps;vz(i)+normrnd(0,sigma_velocity)];
    sample = mvnrnd([0 0],[sigma_h 0;0 sigma_h],1);
    x_gps = [x_gps;x(i)+sample(1)];
    y_gps = [y_gps;y(i)+sample(2)];
    h_gps = [h_gps;h(i)+normrnd(0,sigma_v)];
end


%% Direct KF for Position and Velocity Estimation
x0 = [0;0;10;0;0;0];
P0 = diag(1e0*ones(6,1));
z_imu = [x_imu,y_imu,h_imu,vx_imu,vy_imu,vz_imu];
z_gps = [x_gps,y_gps,h_gps,vx_gps,vy_gps,vz_gps];
Y1 = PVKF(x0,P0,[ax_imu,ay_imu,az_imu],z_gps, ...
        sigma_h,sigma_v,sigma_velocity,sigma_a,1/f_imu,1/f_gps,t(end));
x_KF = Y1(:,1);
y_KF = Y1(:,2);
h_KF = Y1(:,3);
Y2 = DPVKF(zeros(6,1),P0,z_imu,z_gps, ...
        sigma_h,sigma_v,sigma_velocity,sigma_a,1/f_imu,1/f_gps,t(end));
x_DKF = Y2(:,1);
y_DKF = Y2(:,2);
h_DKF = Y2(:,3);


%% Plot
plot3(x,y,h,x_imu,y_imu,h_imu);
xlabel('East');
ylabel('North');
zlabel('Height');
axis([0 105 -5 205 0 20]);
axis equal;
grid minor;
hold on;
plot3(x_gps,y_gps,h_gps,'o');
hold on;
plot3(x_KF,y_KF,h_KF,'x',x_DKF,y_DKF,h_DKF,'^');
legend('Actual Path','IMU Measurement','GPS Measurement', ...
    'KF Estimation','DKF Estimation');

