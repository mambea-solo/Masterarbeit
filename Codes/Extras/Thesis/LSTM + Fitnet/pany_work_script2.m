clear all;
clc;
% IMU parameters
ref_omeg    = 1;                    % MEMS reference rotation in [rad/s]
f_imu           = 200;            % IMU sample rate in [Hz]

set(0,'defaultaxeslinewidth',1.5);
set(0,'defaultlinelinewidth',1.5);
set(0,'DefaultAxesFontSize', 12);

fig_dir = '/MATLAB Drive/Thesis/Figure Plots/'; % to the saving folder
simulation      = adapted_STEP_GYRORATES(f_imu, ref_omeg);
%% collected data from simulation
% gyroscope values
gyroNoisy           = simulation.g_xup;
gyro                = simulation.gyro_reference;

% Deterministic and Stochastic errors
rateSteps           = simulation.rateSteps;
bias_gyro           = simulation.bias_gyro;
SF_gyro             = simulation.SF_gyro;
ARW                 = simulation.ARW;
%% 
i = 1;
j = 0;
% % make the mean 60 times (ratesteps) - this should be the loop probably 
% %     i upto i + 390 + j;
% %     j = -10;
% %     i = i + 410;
% % 
g_xup   = zeros(length(rateSteps),1);   % Noisy mean values in each step
gyr_ref = zeros(length(rateSteps),1);   % Reference mean values in each step
for ii = 1:length(rateSteps)
    g_xup(ii,1)     = mean(gyroNoisy(i:i+390+j));
    gyr_ref(ii,1)   = mean(gyro(i:i+390+j));
    j               = -10;
    i               = i + 410;
    if ii > 1
        i = i + j;
    end
end
%% Putting the data into fitnet

g_xup           = g_xup';
gyr_ref         = gyr_ref';

% Choose a Training Function
trainFcn = 'trainlm';  % Levenberg-Marquardt backpropagation.

% Create a Fitting Network
hiddenLayerSize         = 1;
net                     = fitnet(hiddenLayerSize,trainFcn); % net fitting
% net                     = feedforwardnet(hiddenLayerSize,trainFcn); % feed forward

% Setup Division of Data for Training, Validation, Testing
net.divideParam.trainRatio  = 60/100;
net.divideParam.valRatio    = 20/100;
net.divideParam.testRatio   = 20/100;

% Train the Networks
[netgyr,tr_gyr] = train(net,g_xup,gyr_ref);

% Test the Networks
gyr_netfit          = netgyr(gyroNoisy');
% e_gyr               = gsubtract(gyr_ref,gyr_netfit);
% mse_gyr             = perform(netgyr,gyr_ref,gyr_netfit);

%% Plots
fig1=figure;
plot(gyr_netfit,'b');
hold on;
plot(gyro', 'r');
plot(gyroNoisy','g');
grid on
xlabel('time [s]');
ylabel('gyroscope [rad/s]');
legend('Predicted','Reference','signal from simulation');
title('Gyroscope _ direction');
% saveas(fig1, fullfile(fig_dir,strcat('Gyroscope _ direction', '.png')));
exportgraphics(fig1,'Gyroscope _ direction.png','Resolution',300);

delta_g = gyr_netfit-gyro';
bias_gyro = repelem(bias_gyro,length(delta_g));
fig2=figure;
plot(delta_g,'b');
hold on
plot(bias_gyro,'r');
plot(-1*bias_gyro,'r');
grid on
xlabel('time [s]');
ylabel('gyroscope [rad/s]');
legend('Predicted - Reference','bias');
title('Gyroscope _ direction');
% saveas(fig2, fullfile(fig_dir,strcat('Gyroscope Predicted - Reference', '.png')));
exportgraphics(fig2,'Gyroscope Predicted - Reference.png','Resolution',300);