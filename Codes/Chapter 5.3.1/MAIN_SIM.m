function  simulation = MAIN_SIM(t,f_imu,ref_grav, ref_omeg,num_train_seq) 
%%==========================================================================
%% Main script to run the simuation of the six-position static test (SPST)
%===========================================================================

% clear
% close all

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
% % ref_w = 7.292115*10^-5;                         % [rad/s];
% ref_w    = 1;                                     % reference rotation in [rad/s]
% ref_g    = 9.81;                                  % reference gravity  in [m/s2]
                                             
% f       = 10;                                  % sampling frequency of IMU [hz]
% tspan   = 900;                                 % simulation duration each position [sec] --> 15 min

ref_w    = ref_omeg;
ref_g    = ref_grav;
f        = f_imu;                                 % sampling frequency of IMU [hz]
tspan    = length(t)-1;                           % simulation duration each position [sec] --> 15 min (MUST BE A SCALAR)
sequence = num_train_seq;
%% simulate six position static test
SPST = sim_calibSPST(tspan,f, sequence, ref_g, ref_w, Xsens_spec, 'different');

%% plot signal to control 

% figure('Name','accelerometer calib sequences')
% subplot(3,2,1)
% plot(SPST.time, SPST.acc_xup(:,1))
% title('x_{up}')
% grid on
% subplot(3,2,2)
% plot(SPST.time, SPST.acc_xdow(:,1))
% title('x_{down}')
% grid on
% subplot(3,2,3)
% plot(SPST.time, SPST.acc_yup(:,2))
% title('y_{up}')
% grid on
% subplot(3,2,4)
% plot(SPST.time, SPST.acc_ydow(:,2))
% title('y_{down}')
% grid on
% subplot(3,2,5)
% plot(SPST.time, SPST.acc_zup(:,3))
% title('z_{up}')
% xlabel('time [sec]')
% grid on
% subplot(3,2,6)
% plot(SPST.time, SPST.acc_zdow(:,3))
% title('z_{down}')
% xlabel('time [sec]')
% grid on
% 
% 
% figure('Name','gyro calib sequences')
% subplot(3,2,1)
% plot(SPST.time, SPST.gyr_xup(:,1))
% title('x_{up}')
% grid on
% subplot(3,2,2)
% plot(SPST.time, SPST.gyr_xdow(:,1))
% title('x_{down}')
% grid on
% subplot(3,2,3)
% plot(SPST.time, SPST.gyr_yup(:,2))
% title('y_{up}')
% grid on
% subplot(3,2,4)
% plot(SPST.time, SPST.gyr_ydow(:,2))
% title('y_{down}')
% grid on
% subplot(3,2,5)
% plot(SPST.time, SPST.gyr_zup(:,3))
% title('z_{up}')
% xlabel('time [sec]')
% grid on
% subplot(3,2,6)
% plot(SPST.time, SPST.gyr_zdow(:,3))
% title('z_{down}')
% xlabel('time [sec]')
% grid on

%% Get the bias and scale factor
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%%% Bias calculation is based on (l_up + l_down)/2
%%% SF calculation is based on (l_up - l_down - 2K)/2K
%%% where l_up and l_down are the sensitive axis up and down respectively
%%% and K is the reference known signal, either local gravity constant or
%%% magnitude of Earth rotation rate.
% simulation.b_gyro = zeros(1,3);  % bias value for gyroscope
% simulation.sf_gyro = zeros(1,3); % scale factor for gyroscope
% simulation.b_acc = zeros(1,3);   % bias value for accelerometer
% simulation.sf_acc = zeros(1,3);  % scale factor for accelerometer
% 
% simulation.b_gyro(1,1) = mean((SPST.gyr_xup(:,1)+SPST.gyr_xdow(:,1))/2);
% simulation.b_gyro(1,2) = mean((SPST.gyr_yup(:,2)+SPST.gyr_ydow(:,2))/2);
% simulation.b_gyro(1,3) = mean((SPST.gyr_zup(:,3)+SPST.gyr_zdow(:,3))/2);
% 
% simulation.sf_gyro(1,1) = mean((SPST.gyr_xup(:,1)-SPST.gyr_xdow(:,1)-(2*ref_w))/(2*ref_w));
% simulation.sf_gyro(1,2) = mean((SPST.gyr_yup(:,2)-SPST.gyr_ydow(:,2)-(2*ref_w))/(2*ref_w));
% simulation.sf_gyro(1,3) = mean((SPST.gyr_zup(:,3)-SPST.gyr_zdow(:,3)-(2*ref_w))/(2*ref_w));
% 
% simulation.b_acc(1,1) = mean((SPST.acc_xup(:,1)+SPST.acc_xdow(:,1))/2);
% simulation.b_acc(1,2) = mean((SPST.acc_yup(:,2)+SPST.acc_ydow(:,2))/2);
% simulation.b_acc(1,3) = mean((SPST.acc_zup(:,3)+SPST.acc_zdow(:,3))/2);
% 
% simulation.sf_acc(1,1) = mean((SPST.acc_xup(:,1)-SPST.acc_xdow(:,1)-(2*ref_g))/(2*ref_g));
% simulation.sf_acc(1,2) = mean((SPST.acc_yup(:,2)-SPST.acc_ydow(:,2)-(2*ref_g))/(2*ref_g));
% simulation.sf_acc(1,3) = mean((SPST.acc_zup(:,3)-SPST.acc_zdow(:,3)-(2*ref_g))/(2*ref_g));

% % bias_acc = Xsens_spec.acc_cst(1) * 1e-6 * ref_g;    % m/s^2;
% % SF_acc   = Xsens_spec.acc_cst(2) * 0.01;
% % bias_gyr = Xsens_spec.gyro_cst(1) * (pi/(180*3600));    % [rad/s];
% % SF_gyr   = Xsens_spec.gyro_cst(2) * 0.01;
% % BW       = f;                                 
% % VRW      = Xsens_spec.acc_rand(1) *1e-6 * ref_g *sqrt(BW);
% % ARW      = (Xsens_spec.gyro_rand(1)*60 *sqrt(BW) * pi)/(180*3600); % [rad/s]

%% collected data from simulation
simulation.g_xup = zeros(tspan*f,sequence);
simulation.g_xdow = zeros(tspan*f,sequence);
simulation.g_yup = zeros(tspan*f,sequence);
simulation.g_ydow = zeros(tspan*f,sequence);
simulation.g_zup = zeros(tspan*f,sequence);
simulation.g_zdow = zeros(tspan*f,sequence);

simulation.a_xup = zeros(tspan*f,sequence);
simulation.a_xdow = zeros(tspan*f,sequence);
simulation.a_yup = zeros(tspan*f,sequence);
simulation.a_ydow = zeros(tspan*f,sequence);
simulation.a_zup = zeros(tspan*f,sequence);
simulation.a_zdow = zeros(tspan*f,sequence);

simulation.bias_gyro        = SPST.bias_gyr;
simulation.bias_acc         = SPST.bias_acc;
simulation.SF_gyro          = SPST.SF_gyr;
simulation.SF_acc           = SPST.SF_acc;
simulation.VRW              = SPST.VRW;
simulation.ARW              = SPST.ARW;


for k = 1:sequence % picking the ## column in every cell
%~~~~~~~~~~~~(trial calculation hidden)~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % a = SPST.gyr_xup{k}; %I think this will only hold the kth cell data in the end.
    % simulation.g_xup(:,k)     = a(:,1);

    % gyroscope calculation
    simulation.g_xup(:,k)       = SPST.gyr_xup{k}(:,1); 
    simulation.g_xdow(:,k)      = SPST.gyr_xdow{k}(:,1);
    simulation.g_yup(:,k)       = SPST.gyr_yup{k}(:,2);
    simulation.g_ydow(:,k)      = SPST.gyr_ydow{k}(:,2);
    simulation.g_zup(:,k)       = SPST.gyr_zup{k}(:,3);
    simulation.g_zdow(:,k)      = SPST.gyr_zdow{k}(:,3);

    simulation.acc_xup(:,k)     = SPST.acc_xup{k}(:,1);
    simulation.acc_xdow(:,k)    = SPST.acc_xdow{k}(:,1);
    simulation.acc_yup(:,k)     = SPST.acc_yup{k}(:,2);
    simulation.acc_ydow(:,k)    = SPST.acc_ydow{k}(:,2);
    simulation.acc_zup(:,k)     = SPST.acc_zup{k}(:,3);
    simulation.acc_zdow(:,k)    = SPST.acc_zdow{k}(:,3);
end
end