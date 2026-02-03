
clear
close all
clc

%% simulation szenario:
% An IMU mounted on a turntable which moves at 10 deg steps around the z-axis
% while resting for 10 seconds. The rotation starts with zero postion means x-axis points tword north direction and
% going clock wise while making stops every 10 deg for couple of seconds

% load simulated data by the IMU simulator of Mohamed Bochkati

load sim_fixed_gyro_pos
t=OBS.time;

%1. plot simulation profile
figure
subplot(2,1,1)
plot(t,OBS.attitude_GT.psi)
grid on
ylabel('heading [deg]')
subplot(2,1,2)
plot(t,OBS.attitude_GT.psi_dot)
xlabel('time [sec]')
ylabel('heading rates [deg/s]')

%2. IMU erro model
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% spec for @ian
%--------------------------------------------

Xsens_spec.gyro_cst   = [700 ,    0.15];      % only bias [deg/h)] and scale factor [%]
Xsens_spec.gyro_rand  = [1   , 10, 0.5];      % random erros: ARW (angle white noise) [deg/sqrt(h)] and bias instabilty modeld [deg/h] as GM-1st-order and corr. time [hour]

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% spec for @elsie
%--------------------------------------------

iFOG_spec.gyro_cst    = [1,       0.01];      % only bias [deg/h] and scale factor [%]
iFOG_spec.gyro_rand   = [0.15,    0.07];      % random erros: ARW (angle white noise) [deg/sqrt(h)] and bias instabilty modeld [deg/h]


%% convert spec units for m/s² or rad/s
% constant error

bias_gyro = Xsens_spec.gyro_cst(1) * (pi/(180*3600));    % [rad/s];
SF_gyro   = Xsens_spec.gyro_cst(2) * 0.01;

% random error
BW=200;        % assuming 200 Hz of IMU-band width
ARW   = (Xsens_spec.gyro_rand(1)*60 *sqrt(BW) * pi)/(180*3600); % [rad/s]


% define random vector for both wx and wy

WNx  =randn(length(t),1);            % random vector to simulate imu spec
WNy  =randn(length(t),1);            % random vector to simulate imu spec
WNz  =randn(length(t),1);            % random vector to simulate imu spec




%% generate noisy signals for north finding technique assmuning same constant error for all three gyro axis

wx_noisy= OBS.omega_ib_b_GT(:,1) *(1+SF_gyro) + bias_gyro + ARW* WNx;
wy_noisy= OBS.omega_ib_b_GT(:,2)*(1+SF_gyro) + bias_gyro + ARW* WNy;
wz_noisy= OBS.omega_ib_b_GT(:,3)*(1+SF_gyro) + bias_gyro + ARW* WNz;

figure
subplot(3,1,1)
plot( t, wx_noisy)
hold on
plot( t,OBS.omega_ib_b_GT(:,1))
ylabel('w_x [rad/sec]')
subplot(3,1,2)
plot( t, wy_noisy)
hold on
plot( t,OBS.omega_ib_b_GT(:,2))
subplot(3,1,3)
plot( t, wz_noisy)
hold on
plot( t,OBS.omega_ib_b_GT(:,3))
xlabel('time [sec]')
ylabel('w_y [rad/sec]')





