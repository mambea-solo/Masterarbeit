classdef StrapdownRegressionLayer_modified < nnet.layer.RegressionLayer
    % regression layer to evaluate rotation rates estimates against known
    % attitude values
    % does account for the fact, that the initial attitude is unknown, i.e.
    % the loss functions conducts internally some kind of coarse alignment
    properties 
        f_imu; % assumed IMU sample rate 
    end

    methods
        function layer = StrapdownRegressionLayer_modified(name)
            % Set layer name.
            layer.Name = name;
            layer.f_imu=200; % assumed IMU sample rate [note: for NN training is actually irrelevant; it is just kept here for didactic purposes]
            
            % Set layer description.
            layer.Description = 'using the NN output, an 1D attitude strapdown computation is performed and is matched to know attitude values';
        end
        
        function loss = forwardLoss(layer,Y, T)
            % Y ... calibrated rotation ratesa in [rad/s] as output by the NN; 
            %       index scheme Y(x,y,z): x ... 1, y ... trajectory index,
            %       z ... time epoch index
      
            % T ... reference attitude values [rad]
            global rowsnotNeeded gyroscale

            
            %integrate rotation rates into attiude, note: '3' is the time
            %dimension of the input data
            %note: the integration starts here at 0 deg, but actually the
            %intial values is irrelevant as psi0 accounts for any offset
            att = cumsum(Y*gyroscale,3)/layer.f_imu; 

            % poor mans coarse alignement computes the initial angular
            % value as the mean between the true angular values and the
            % ones from the strapdown computation (=cumsum)
            psi0= abs(mean( att-T ,3)); % note: 3 indicates the dimension along the mean is computed
                                   % third index counts the different test
                                   % data sets
            
            % apply now the coarse alignment to the result from the
            % strapdown computation
            % since we cover multiple training sets in one step, care must
            % be taken to ensure proper indexing
            att = att + repmat( psi0, [1, 1, size(att,3)]);
            
            res = att-T; % difference between predicted and reference attitude
            res = reshape(res,[1,size(res,3)]);
            res(rowsnotNeeded) = 0;         % puts values in the index rowsnotNeeded (transition regions) to be zero.

            % compute the L2 norm between computed and reference attitude
            SSE = sum((res').^2); %L2-attitude norm for each trajectory
            %SSE = sum(SSE); % sum over all trajectories
            
            % return loss function
            loss=SSE;

        end

         function dLdY = backwardLoss(layer,Y,T)
            % dLdY = backwardLoss(layer, Y, T) returns the derivative of
            % the loss function L, with respect to the predictions Y.
            % L= SSE equation
            %
            % in our case we need to compute the derivative of the L2
            % attitude norm w.r.t. to the rotation rates as produced by the
            % NN.
            %
            % Y, T ... see definition of forwardLoss
            global rowsnotNeeded gyroscale

            %integrate rotation rates into attiude, note: '3' is the time
            %dimension of the input data
            att = cumsum(Y*gyroscale,3)/layer.f_imu;
            
            % apply coarse alignment also the purpose of computing the first derivate of the loss function 
            psi0=abs(mean( att-T,3));
            att = att + repmat( psi0, [1, 1, size(att,3)]);
            
            % compute first derivative (see handwritten notes)
            dLdY = cumsum(2*(att-T),3,'reverse')/layer.f_imu^2; 

        end
    end
end