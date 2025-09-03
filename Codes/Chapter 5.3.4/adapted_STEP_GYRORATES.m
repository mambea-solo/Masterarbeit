function  simulation = adapted_STEP_GYRORATES(f_imu, ~) 
% 
clear
close all

%% simulation szenario:
% An IMU mounted on a turntable which increses the rotation rates around the z-axis
% to achieve constant rotation rates while resting for 10 seconds at each
% rate value. The simulation covers both negative and positive angle
% velocity rates, i.e. from -1 rad/s --Y + 1 rad/s

% IMU parameters
f_imu     =200; % IMU sample rate in [Hz]

minRate=-1 *180/pi;       % deg/s
maxRate= 1 *180/pi;       % deg/s

% set up gyro rates steps 
rateSteps = linspace(minRate,maxRate,60);

stopDuration = 2;                  %sec
gyro=[];

for i=1:length(rateSteps)
   
    gyro_tem= ones(stopDuration *f_imu ,1) * rateSteps(i);
    gyro=[gyro;gyro_tem];

end

% smooth transition section betwee rate step to avoid harsh transition
gyro= smoothdata(gyro,"gaussian",20);

figure
plot(gyro)
hold on

% M = movmean(A,3)
t=(1/f_imu:1/f_imu:length(rateSteps)* stopDuration)' ; % considered time span for training data; here 50 s

figure
plot(t,gyro)
xlabel('time [sec]')
ylabel('\omega_y [deg/sec]')
grid on

% convert signal to rad/s                     %  [rad/s]
gyro=gyro*pi/180;

% ref_omeg    = 7.292115*10^-5;       % Earth's rotation [rad/s];
ref_omeg    = 1;                    % MEMS reference rotation in [rad/s]
WN  =randn(length(t),1);            % random vector to simulate imu spec

% plot error-free reference signal

figure
plot(t,gyro)
xlabel('time [sec]')
ylabel('\omega_z [rad/sec]')
grid on

%1. IMU erro model
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


%% generate signals
gyroNoisy  = gyro  *(1+SF_gyro) + bias_gyro + ARW* WN;  % x up

% plot noisy biased signals

figure
plot(t,gyro)
hold on
plot(t,gyroNoisy)
xlabel('time [sec]')
ylabel('\omega_z [rad/sec]')
legend('reference','noisy')
grid on

%% collected data from simulation  
simulation.g_xup            = gyroNoisy; 
simulation.gyro_reference   = gyro; 
simulation.bias_gyro        = bias_gyro;
simulation.SF_gyro          = SF_gyro;
simulation.ARW              = ARW;
simulation.rateSteps        = rateSteps;

end