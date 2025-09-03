function  simulation = MAIN_SIM_DYN_CALIB(t,f_acc,amp_acc, amp_gyro,ref_grav,num_train_seq) 
%%==========================================================================
%% Main script to run the dynamic simuation of the IMU
%===========================================================================
    % clear
    % close all
    
    % IMU parameters
    % f               = f_imu;        % IMU sample rate in [Hz]
    tspan           = length(t);    % simulation duration each position [sec] --> 50 sec (MUST BE A SCALAR)
    sequence        = num_train_seq;

    % amp_acc         = 4;                      % [m/s²]
    % amp_gyro        = 50;                     % [deg/s]
    amp_gyro         = amp_gyro * pi/180;            % [rad/s]    
    % f_acc           = 2;                      % [Hz]
    
    ref_g           = ref_grav;               % [m/s2], only for error unit converstion 
   
    % generate acc and gyro signal
    
    acc.x           = amp_acc * sin(2*pi*f_acc*t);
    acc.x           = acc.x';
    gyro.y          = amp_gyro * sin(2*pi*f_acc*t);
    gyro.y          = gyro.y';      

    % Normalization of signal
    % % first scale it to 0 to 1
    acc_range_x     = max(acc.x(:)) - min(acc.x(:));
    gyro_range_y    = max(gyro.y(:)) - min(gyro.y(:));
    % %then put the range to -1 to 1
    acc.x           = (acc.x - min(acc.x(:))) / acc_range_x;
    gyro.y          = (gyro.y - min(gyro.y(:))) / gyro_range_y;

    % plot error-free reference signal  
    % figure(1)
    % subplot(2,1,1)
    % plot(t,acc.x)
    % ylabel('f_x [m/s2]')
    % grid on
    % subplot(2,1,2)
    % plot(t,gyro.y)
    % xlabel('time [sec]')
    % ylabel('\omega_y [deg/sec]')
    % grid on
    
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
    
    
    %% convert spec units for m/s² or rad/s
    % constant error
    bias_acc        = Xsens_spec.acc_cst(1) * 1e-6 * ref_g;    % m/s^2;
    SF_acc          = Xsens_spec.acc_cst(2) * 0.01;
    
    bias_gyro       = Xsens_spec.gyro_cst(1) * (pi/(180*3600));    % [rad/s];
    SF_gyro         = Xsens_spec.gyro_cst(2) * 0.01;
    
    % random error
    BW              = 200;        % assuming 200 Hz of IMU-band width
    VRW             = Xsens_spec.acc_rand(1) *1e-6 * ref_g *sqrt(BW);
    ARW             = (Xsens_spec.gyro_rand(1)*60 *sqrt(BW) * pi)/(180*3600); % [rad/s]
    
    
    %% generate noisy signals
    % 
    % acc.x_noisy     = acc.x   *(1+SF_acc)  + bias_acc  + VRW* WN;  % x up
    % gyro.y_noisy    = gyro.y  *(1+SF_gyro) + bias_gyro + ARW* WN;  % x up
    % 
    % % Normalization of noisy signal
    % % % first scale it to 0 to 1
    % acc_range_x     = max(acc.x_noisy(:)) - min(acc.x_noisy(:));
    % gyro_range_y    = max(gyro.y_noisy(:)) - min(gyro.y(:));
    % % %then put the range to -1 to 1
    % acc.x_noisy     = (acc.x_noisy  - min(acc.x_noisy (:))) / acc_range_x;
    % gyro.y_noisy    = (gyro.y_noisy - min(gyro.y_noisy(:))) / gyro_range_y;
    % 
    % % plot noisy biased signals 
    % figure(1)
    % subplot(2,1,1)
    % hold on
    % plot(t,acc.x_noisy)
    % ylabel('f_x [m/s2]')
    % legend('reference','noisy')
    % grid on
    % subplot(2,1,2)
    % hold on
    % plot(t,gyro.y_noisy)
    % xlabel('time [sec]')
    % ylabel('\omega_y [deg/sec]')
    % legend('reference','noisy')
    % grid on

    %% collected data from simulation
    simulation.g_xup = zeros(tspan,sequence);

    simulation.a_xup = zeros(tspan,sequence);

    for k = 1:sequence % picking the ## column in every cell
        WN              = randn(tspan,1);         % random vector to simulate imu spec

        acc.x_noisy     = acc.x   *(1+SF_acc)  + bias_acc  + VRW* WN;  % x up
        gyro.y_noisy    = gyro.y  *(1+SF_gyro) + bias_gyro + ARW* WN;  % x up
    
        % Normalization of noisy signal
        % % first scale it to 0 to 1
        acc_range_x     = max(acc.x_noisy(:)) - min(acc.x_noisy(:));
        gyro_range_y    = max(gyro.y_noisy(:)) - min(gyro.y(:));
        % %then put the range to -1 to 1
        acc.x_noisy     = (acc.x_noisy  - min(acc.x_noisy (:))) / acc_range_x;
        gyro.y_noisy    = (gyro.y_noisy - min(gyro.y_noisy(:))) / gyro_range_y;

        % acc.x_noisy     = acc.x   *(1+SF_acc)  + bias_acc  + VRW* WN;  % x up
        % gyro.y_noisy    = gyro.y  *(1+SF_gyro) + bias_gyro + ARW* WN;  % x up
        
        % gyroscope calculation
        simulation.g_xup(:,k)       = gyro.y_noisy(:,1);
        
        simulation.acc_xup(:,k)     = acc.x_noisy(:,1);

    end
    
    % more values to be used in main script MEMS
    simulation.acc_reference    = acc.x;
    simulation.gyro_reference   = gyro.y; 
    simulation.bias_gyro        = bias_gyro;
    simulation.bias_acc         = bias_acc;
    simulation.SF_gyro          = SF_gyro;
    simulation.SF_acc           = SF_acc;
    simulation.VRW              = VRW;
    simulation.ARW              = ARW;
% % end


