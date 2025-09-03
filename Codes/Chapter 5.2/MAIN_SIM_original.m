
%%==========================================================================
%% Main script to run the simuation of the six-position static test (SPST)
%===========================================================================

clear
close all

set(0,'defaultaxeslinewidth',1.5);
set(0,'defaultlinelinewidth',1.5);
set(0,'DefaultAxesFontSize', 12);
%% define simulation paramter

%1. IMU erro model
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% spec for @ian
%--------------------------------------------
Xsens_spec.acc_cst    = [5000, 1      ];      % only constant bias [ug] and scale factor    [%]
Xsens_spec.acc_rand   = [60  , 15, 0.5];      % random erros: VRW (accl white noise) [µg/sqrt(Hz)] and bias instabilty modeld [µg] as GM-1st-order and corr. time [hour]

Xsens_spec.gyro_cst   = [700 ,    0.15];      % only bias [deg/h)] and scale factor [%]
Xsens_spec.gyro_rand  = [1   , 10, 0.5];      % random erros: ARW (angle white noise) [deg/sqrt(h)] and bias instabilty modeld [deg/h] as GM-1st-order and corr. time [hour]

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% spec for @elsie
%--------------------------------------------
iFOG_spec.acc_cst     = [100,     0.01];      % only bias [ug] and scale factor    [%]
iFOG_spec.acc_rand    = [50,        10];      % random erros: VRW (accl white noise) [µg/sqrt(Hz)] and bias instabilty modeld [µg] 

iFOG_spec.gyro_cst    = [1,       0.01];      % only bias [deg/h] and scale factor [%]
iFOG_spec.gyro_rand   = [0.15,    0.07];      % random erros: ARW (angle white noise) [deg/sqrt(h)] and bias instabilty modeld [deg/h]

%%  define reference signal and signla related parameter
% ref_w=7.292115*10^-5; % [rad/s];
ref_w=1; % [rad/s];                           % reference rotation in [rad/s]
ref_g=9.81;                                   % reference gravity  in [m/s2]
                                             
tspan=900;                                    % simulation duration each position [sec] --> 15 min
f=10;                                         % sampling frequency of IMU [hz]

%% simulate six position static test
SPST = sim_calibSPST_original(tspan,f, ref_g, ref_w, Xsens_spec, 'different');

%% plot signal to control 

fig1=figure('Name','accelerometer calib sequences');
subplot(3,2,1)
plot(SPST.time, SPST.acc_xup(:,1))
title('x_{up}')
ylabel('m/s^2')
grid on
subplot(3,2,2)
plot(SPST.time, SPST.acc_xdow(:,1))
title('x_{down}')
ylabel('m/s^2')
grid on
subplot(3,2,3)
plot(SPST.time, SPST.acc_yup(:,2))
title('y_{up}')
ylabel('m/s^2')
grid on
subplot(3,2,4)
plot(SPST.time, SPST.acc_ydow(:,2))
title('y_{down}')
ylabel('m/s^2')
grid on
subplot(3,2,5)
plot(SPST.time, SPST.acc_zup(:,3))
title('z_{up}')
xlabel('time [sec]')
ylabel('m/s^2')
grid on
subplot(3,2,6)
plot(SPST.time, SPST.acc_zdow(:,3))
title('z_{down}')
xlabel('time [sec]')
ylabel('m/s^2')
grid on
exportgraphics(fig1,'accelerometer calib sequences.png','Resolution',300);


fig2=figure('Name','gyro calib sequences');
subplot(3,2,1)
plot(SPST.time, SPST.gyr_xup(:,1))
title('x_{up}')
ylabel('rad/s')
grid on
subplot(3,2,2)
plot(SPST.time, SPST.gyr_xdow(:,1))
title('x_{down}')
ylabel('rad/s')
grid on
subplot(3,2,3)
plot(SPST.time, SPST.gyr_yup(:,2))
title('y_{up}')
ylabel('rad/s')
grid on
subplot(3,2,4)
plot(SPST.time, SPST.gyr_ydow(:,2))
title('y_{down}')
ylabel('rad/s')
grid on
subplot(3,2,5)
plot(SPST.time, SPST.gyr_zup(:,3))
title('z_{up}')
xlabel('time [sec]')
ylabel('rad/s')
grid on
subplot(3,2,6)
plot(SPST.time, SPST.gyr_zdow(:,3))
title('z_{down}')
xlabel('time [sec]')
ylabel('rad/s')
grid on
exportgraphics(fig2,'gyro calib sequences.png','Resolution',300);

%% Calculation of SPST using the formula
simulation.b_gyro = zeros(1,3);  % bias value for gyroscope
simulation.sf_gyro = zeros(1,3); % scale factor for gyroscope
simulation.b_acc = zeros(1,3);   % bias value for accelerometer
simulation.sf_acc = zeros(1,3);  % scale factor for accelerometer

simulation.b_gyro(1,1) = mean((SPST.gyr_xup(:,1)+SPST.gyr_xdow(:,1))/2);
simulation.b_gyro(1,2) = mean((SPST.gyr_yup(:,2)+SPST.gyr_ydow(:,2))/2);
simulation.b_gyro(1,3) = mean((SPST.gyr_zup(:,3)+SPST.gyr_zdow(:,3))/2);

simulation.sf_gyro(1,1) = mean((SPST.gyr_xup(:,1)-SPST.gyr_xdow(:,1)-(2*ref_w))/(2*ref_w));
simulation.sf_gyro(1,2) = mean((SPST.gyr_yup(:,2)-SPST.gyr_ydow(:,2)-(2*ref_w))/(2*ref_w));
simulation.sf_gyro(1,3) = mean((SPST.gyr_zup(:,3)-SPST.gyr_zdow(:,3)-(2*ref_w))/(2*ref_w));

simulation.b_acc(1,1) = mean((SPST.acc_xup(:,1)+SPST.acc_xdow(:,1))/2);
simulation.b_acc(1,2) = mean((SPST.acc_yup(:,2)+SPST.acc_ydow(:,2))/2);
simulation.b_acc(1,3) = mean((SPST.acc_zup(:,3)+SPST.acc_zdow(:,3))/2);

simulation.sf_acc(1,1) = mean((SPST.acc_xup(:,1)-SPST.acc_xdow(:,1)-(2*ref_g))/(2*ref_g));
simulation.sf_acc(1,2) = mean((SPST.acc_yup(:,2)-SPST.acc_ydow(:,2)-(2*ref_g))/(2*ref_g));
simulation.sf_acc(1,3) = mean((SPST.acc_zup(:,3)-SPST.acc_zdow(:,3)-(2*ref_g))/(2*ref_g));

bias_acc = Xsens_spec.acc_cst(1) * 1e-6 * ref_g;    % m/s^2;
SF_acc   = Xsens_spec.acc_cst(2) * 0.01;
bias_gyr = Xsens_spec.gyro_cst(1) * (pi/(180*3600));    % [rad/s];
SF_gyr   = Xsens_spec.gyro_cst(2) * 0.01;
BW       = f;                                 
VRW      = Xsens_spec.acc_rand(1) *1e-6 * ref_g *sqrt(BW);
ARW      = (Xsens_spec.gyro_rand(1)*60 *sqrt(BW) * pi)/(180*3600); % [rad/s]
