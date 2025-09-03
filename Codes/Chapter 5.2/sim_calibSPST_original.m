function  SPST = sim_calibSPST_original(tspan,f, ref_g, ref_w, imu_spec, asmp)
%SIM_CALIBSPST Summary of this function goes here
%   Detailed explanation goes here

% t_span  [sec]     = time duration for each calibration position, ususally 15 minutes
% f       [Hz]      = sampling frequency of the simulated signal
% ref_g   [m/s²]    = reference gravity 
% ref_w   [rad/s]   = reference rotation, in case of high quality gyro,
%                     earth rotation cosntant, in case of low-cost MEMS, artifical rotation hast to be introduced  
% imu_spec          = imu error specificationas deined in the main script
% asmmp   [str      = asmmuption if all axis contains the same 'equal'
%                     noise vector or different 'different'

% define simulation
g       =ref_g;   % reference gravity
omeg    =ref_w;   % reference rotation
sig_len =tspan * f;


%% convert spec units for m/s² or rad/s
 % constant error
  bias_acc = imu_spec.acc_cst(1) * 1e-6 * ref_g;    % m/s^2;
  SF_acc   = imu_spec.acc_cst(2) * 0.01;

  bias_gyr = imu_spec.gyro_cst(1) * (pi/(180*3600));    % [rad/s];
  SF_gyr   = imu_spec.gyro_cst(2) * 0.01;

  % random error
  BW=200;        % assuming 200 Hz of IMU-band width
  VRW   = imu_spec.acc_rand(1) *1e-6 * ref_g *sqrt(BW);
  ARW   = (imu_spec.gyro_rand(1)*60 *sqrt(BW) * pi)/(180*3600); % [rad/s]


%% generate signals


% simulate six potion
WN  =randn(sig_len,1);
WNx =randn(sig_len,1);
WNy =randn(sig_len,1);
WNz =randn(sig_len,1);



%N=1;       %  Non-orthogonality
% NO=  [0  -N   N  ; N   0  -N ; -N   N   0];               % unitless

%% assume all axes have the same noise
switch asmp

case 'equal'
    % accelerometer part
    SPST.acc_xup  = [+g , 0 , 0 ] .*(1+SF_acc) + bias_acc + VRW*[ WN  WN  WN  ];  % x up
    SPST.acc_xdow = [-g , 0 , 0 ] .*(1+SF_acc) + bias_acc + VRW*[ WN  WN  WN  ];  % X down
    SPST.acc_yup  = [ 0 ,+g , 0 ] .*(1+SF_acc) + bias_acc + VRW*[ WN  WN  WN  ];  % Y up
    SPST.acc_ydow = [ 0 ,-g , 0 ] .*(1+SF_acc) + bias_acc + VRW*[ WN  WN  WN  ];  % Y down
    SPST.acc_zup  = [ 0 , 0 ,+g ] .*(1+SF_acc) + bias_acc + VRW*[ WN  WN  WN  ];  % Z up
    SPST.acc_zdow = [ 0 , 0 ,-g ] .*(1+SF_acc) + bias_acc + VRW*[ WN  WN  WN  ];  % Z down
     
    % gyro part

    SPST.gyr_xup  = [+omeg , 0 , 0 ].*(1+SF_gyr) + bias_gyr + ARW*[ WN  WN  WN  ];  % x up
    SPST.gyr_xdow = [-omeg , 0 , 0 ].*(1+SF_gyr) + bias_gyr + ARW*[ WN  WN  WN  ];  % X down
    SPST.gyr_yup  = [ 0 ,+omeg , 0 ].*(1+SF_gyr) + bias_gyr + ARW*[ WN  WN  WN  ];  % Y up
    SPST.gyr_ydow = [ 0 ,-omeg , 0 ].*(1+SF_gyr) + bias_gyr + ARW*[ WN  WN  WN  ];  % Y down
    SPST.gyr_zup  = [ 0 , 0 ,+omeg ].*(1+SF_gyr) + bias_gyr + ARW*[ WN  WN  WN  ];  % Z up
    SPST.gyr_zdow = [ 0 , 0 ,-omeg ].*(1+SF_gyr) + bias_gyr + ARW*[ WN  WN  WN  ];  % Z down

case 'different'
    %% assume all axes have different noise vector
    % accelerometer part
    SPST.acc_xup  = [+g , 0 , 0 ] .*(1+SF_acc)+ bias_acc + VRW*[ WNx  WNy  WNz  ];  % x up
    SPST.acc_xdow = [-g , 0 , 0 ] .*(1+SF_acc)+ bias_acc + VRW*[ WNx  WNy  WNz  ];  % X down
    SPST.acc_yup  = [ 0 ,+g , 0 ] .*(1+SF_acc)+ bias_acc + VRW*[ WNx  WNy  WNz  ];  % Y up
    SPST.acc_ydow = [ 0 ,-g , 0 ] .*(1+SF_acc)+ bias_acc + VRW*[ WNx  WNy  WNz  ];  % Y down
    SPST.acc_zup  = [ 0 , 0 ,+g ] .*(1+SF_acc)+ bias_acc + VRW*[ WNx  WNy  WNz  ];  % Z up
    SPST.acc_zdow = [ 0 , 0 ,-g ] .*(1+SF_acc)+ bias_acc + VRW*[ WNx  WNy  WNz  ];  % Z down
     
    % gyro part

    SPST.gyr_xup  = [+omeg , 0 , 0 ].*(1+SF_gyr) + bias_gyr + ARW*[ WNx  WNy  WNz  ];  % x up
    SPST.gyr_xdow = [-omeg , 0 , 0 ].*(1+SF_gyr) + bias_gyr + ARW*[ WNx  WNy  WNz  ];  % X down
    SPST.gyr_yup  = [ 0 ,+omeg , 0 ].*(1+SF_gyr) + bias_gyr + ARW*[ WNx  WNy  WNz  ];  % Y up
    SPST.gyr_ydow = [ 0 ,-omeg , 0 ].*(1+SF_gyr) + bias_gyr + ARW*[ WNx  WNy  WNz  ];  % Y down
    SPST.gyr_zup  = [ 0 , 0 ,+omeg ].*(1+SF_gyr) + bias_gyr + ARW*[ WNx  WNy  WNz  ];  % Z up
    SPST.gyr_zdow = [ 0 , 0 ,-omeg ].*(1+SF_gyr) + bias_gyr + ARW*[ WNx  WNy  WNz  ];  % Z down
end

SPST.time=[0:1/f:tspan-1/f]';
