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
clc
clear
A1 = 1;
A2 = 2;
L1 = 100;
L2 = 200;
f_imu = 200;              % Sampling frequency of IMU, unit: Hz.
step = 1/f_imu;             % time step


%% Stage 1: East Direction Acceleration
t = (0:step:3.14)';
ax = A1*sin(t);
ay = zeros(length(t),1);
az = zeros(length(t),1);
vx = A1*(1-cos(t));
vy = zeros(length(t),1);
vz = zeros(length(t),1);
x = A1*(t-sin(t));
y = zeros(length(t),1);
h = 10*ones(length(t),1);


%% Stage 2: East Direction Uniform Motion
t = (3.14+step:step:L1/2/A1)';
ax = [ax;zeros(length(t),1)];
ay = [ay;zeros(length(t),1)];
az = [az;zeros(length(t),1)];
vx = [vx;2*A1*ones(length(t),1)];
vy = [vy;zeros(length(t),1)];
vz = [vz;zeros(length(t),1)];
x = [x;A1*3.14+2*A1*(t-3.14)];
y = [y;zeros(length(t),1)];
h = [h;10*ones(length(t),1)];


%% Stage 3: East Direction Deacceleration
t = (L1/2/A1+step:step:L1/2/A1+3.14)';
ax = [ax;-A1*sin(t-L1/2/A1)];
ay = [ay;zeros(length(t),1)];
az = [az;zeros(length(t),1)];
vx = [vx;A1*(cos(t-L1/2/A1)+1)];
vy = [vy;zeros(length(t),1)];
vz = [vz;zeros(length(t),1)];
x = [x;A1*3.14+2*A1*(L1/2/A1-3.14)+A1*(t-L1/2/A1)+A1*sin(t-L1/2/A1)];
y = [y;zeros(length(t),1)];
h = [h;10*ones(length(t),1)];


%% Stage 4: North Direction Acceleration
t = (L1/2/A1+3.14+step:step:L1/2/A1+6.28)';
ax = [ax;zeros(length(t),1)];
ay = [ay;A2*sin(t-L1/2/A1-3.14)];
az = [az;zeros(length(t),1)];
vx = [vx;zeros(length(t),1)];
vy = [vy;A2*(1-cos(t-L1/2/A1-3.14))];
vz = [vz;zeros(length(t),1)];
x = [x;x(end)*ones(length(t),1)];
y = [y;A2*(t-L1/2/A1-3.14-sin(t-L1/2/A1-3.14))];
h = [h;10*ones(length(t),1)];


%% Stage 5: North Direction Uniform Motion
t = (L1/2/A1+6.28+step:step:L1/2/A1+3.14+L2/2/A2)';
ax = [ax;zeros(length(t),1)];
ay = [ay;zeros(length(t),1)];
az = [az;zeros(length(t),1)];
vx = [vx;zeros(length(t),1)];
vy = [vy;2*A2*ones(length(t),1)];
vz = [vz;zeros(length(t),1)];
x = [x;x(end)*ones(length(t),1)];
y = [y;A2*3.14+2*A2*(t-L1/2/A1-6.28)];
h = [h;10*ones(length(t),1)];


%% Stage 6: North Direction Deacceleration
t = (L1/2/A1+3.14+L2/2/A2+step:step:L1/2/A1+L2/2/A2+6.28)';
ax = [ax;zeros(length(t),1)];
ay = [ay;-A2*sin(t-L1/2/A1-3.14-L2/2/A2)];
az = [az;zeros(length(t),1)];
vx = [vx;zeros(length(t),1)];
vy = [vy;A2*(cos(t-L1/2/A1-3.14-L2/2/A2)+1)];
vz = [vz;zeros(length(t),1)];
x = [x;x(end)*ones(length(t),1)];
y = [y;A2*3.14+2*A2*(L2/2/A2-3.14)+A2*(t-L1/2/A1-3.14-L2/2/A2)+...
            A2*sin(t-L1/2/A1-3.14-L2/2/A2)];
h = [h;10*ones(length(t),1)];


%% Visulization
t = (0:step:L1/2/A1+L2/2/A2+6.28)';
plot3(x,y,h);
xlabel('East');
ylabel('North');
zlabel('Height');
axis([0 L1+5 -5 L2+5 0 20]);
grid minor;

save('L_Path.mat','t','ax','ay','az','vx','vy','vz','x','y','h');

