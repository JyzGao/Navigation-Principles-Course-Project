% 四旋翼构型为cX”型，螺旋桨序号如下所示(使用PX4标准"X"型配置)：
%            3↓   1↑
%              \ /
%              / \
%            2↑   4↓
%   其中，↑表示螺旋桨逆时针旋转；↓表示螺旋桨顺时针旋转。
% 参数说明
%   wi：四个螺旋桨的转速(rad/s)
%   c_T：螺旋桨拉力系数(拟合计算得出，本模型中直接使用拉力插值表，不使用c_T)
%   c_M：螺旋桨力矩系数
%   d：机体中心和任一电机的距离(m)
%   f：螺旋桨拉力（机体轴）
%   tau_x：x轴反扭力矩（机体轴） = R*√2/2*(-f1 + f2 + f3 - f4);
%   tau_y：y轴反扭力矩（机体轴） = R*√2/2*(f1 - f2 + f3 - f4);
%   tau_z：z轴反扭力矩（机体轴） = c_M * (w1^2 + w2^2 - w3^2 - w4^2);


%% State Variables Initiation
stateInp.xyz = [0,0,10];
stateInp.uvw = [0,0,0];
stateInp.euler = [0,0,0];
stateInp.pqr = [0,0,0];


%% UAV Parameters
Vehicle.Airframe.mass = 1.612868879;

Ixc = 9373.661914436/1e6;
Iyc = 11755.067703159/1e6;
Izc = 17533.553367555/1e6;
Ixyc = 117.728469266/1e6;
Ixzc = -70.837777552/1e6;
Iyzc = 110.329817672/1e6;
Vehicle.Airframe.inertia = [Ixc 0 0;
                           0 Iyc 0;
                           0 0 Izc];
Vehicle.Airframe.diameter = 0.41;
Vehicle.Airframe.Cdx = 0;   % 空气阻力系数

k_T = 1.097e-5;
k_tau = 1.779e-7;
% ModelParam_motorT = 0.02;
% ModelInit_RPM = 5300;       %552/pi/2*60;


%% Controllers' Parameters
omega_n_theta = 10;
omega_n_phi = 10;
omega_n_yaw = 1;
omega_n_h = 1;
damp_ratio_theta = 0.85;
damp_ratio_phi = 0.85;
damp_ratio_yaw = 0.85;
damp_ratio_h = 0.85;

kp_position = 0.1;
kp_vx = 0.1;
kp_vy = 0.1;
kp_yaw = omega_n_yaw^2*Izc;
kp_h = omega_n_h^2*Vehicle.Airframe.mass;
kp_theta = omega_n_theta^2*Iyc;
kp_phi = omega_n_phi^2*Ixc;

kd_yaw = damp_ratio_yaw*2*sqrt(kp_yaw*Izc);
kd_h = damp_ratio_h*2*sqrt(kp_h*Vehicle.Airframe.mass);
kd_theta = damp_ratio_theta*2*sqrt(kp_theta*Iyc);
kd_phi = damp_ratio_phi*2*sqrt(kp_phi*Ixc);

