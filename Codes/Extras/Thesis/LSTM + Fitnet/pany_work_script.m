clear all;
clc;
% IMU parameters
ref_omeg    = 1;                    % MEMS reference rotation in [rad/s]
f_imu           = 200;            % IMU sample rate in [Hz]

% fig_dir = '/MATLAB Drive/Thesis/Figure Plots/'; % to the saving folder
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
%% Converting the data to cell format for Neural Network 
train_data          = num2cell(g_xup,1);
train_labels        = num2cell(gyr_ref,1);
test_data           = num2cell(gyroNoisy,1);
%% since I need my input size to the NN to be 1, I need to change the
% appearance of the cell array, where each cell of both data & labels
% to be 1xsplit_idx & 1xsplit_id+1:end respectively

train_data{:}       = train_data{:}';
train_labels{:}     = train_labels{:}';
test_data{:}        = test_data{:}';
%% configure neural network (NN)
numFeatures     = size(train_data, 1); % Number of features in your data
numHiddenUnits  = 15; % Adjust as needed
numResponses    = 1;

layers = [
    sequenceInputLayer(numFeatures)
    lstmLayer(numHiddenUnits, 'OutputMode', 'sequence')
    fullyConnectedLayer(numResponses)
    regressionLayer 
];

% Set training options with the Adam optimizer
options = trainingOptions('adam', ...
    'MaxEpochs', 30, ...
    'MiniBatchSize', 2^3, ...
    'SequenceLength', 10, ... % Adjust as needed
    'GradientThreshold', 1, ...
    'InitialLearnRate', 1e-3, ...
    'LearnRateSchedule', 'piecewise', ...
    'LearnRateDropFactor', 0.2, ...
    'LearnRateDropPeriod', 5, ...
    'Shuffle', 'every-epoch', ...  % to reduce biases in training
    'ValidationFrequency',10,...
    'ExecutionEnvironment', 'auto', ...
    'Plots','training-progress',...
    'Verbose',1);

% Train the LSTM network
net = trainNetwork(train_data, train_labels, layers, options);

%% Test the trained network
predicted_labels = predict(net, test_data);
predicted_labels = (cell2mat(predicted_labels))';
%% Plots
fig1=figure;
plot(predicted_labels,'b');
hold on;
plot(gyro, 'r');
plot(gyroNoisy,'g');
xlabel('time [s]');
ylabel('gyroscope [rad/s]');
legend('Predicted','Reference','signal from simulation');
title('Gyroscope _ direction');
% saveas(fig1, fullfile(fig_dir,strcat('Gyroscope _ direction', '.png')));

delta_g = predicted_labels-gyro;
bias_gyro = repelem(bias_gyro,length(delta_g));
fig2=figure;
plot(delta_g,'b');
hold on
plot(bias_gyro,'r');
plot(-1*bias_gyro,'r');
xlabel('time [s]');
ylabel('gyroscope [rad/s]');
legend('Predicted - Reference','bias');
title('Gyroscope _ direction');
% saveas(fig2, fullfile(fig_dir,strcat('Gyroscope Predicted - Reference', '.png')));
