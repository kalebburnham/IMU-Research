function [Acc,Gyr] = addError(Acc,Gyr)
    % Acceleration First
    
    % Global variables
    SensorNoiseSigma = 0.012;
    SensorNoiseMean = 0;
    tau = 1;            % Correlation Coefficient
    c = exp(-1/tau);    % constant defined by Gauss-Markov model
    BiasStability = 0.04;
    BiasSigma = BiasStability/sqrt(tau);                % Sigma sub bias
    BiasVariance = BiasSigma^2;                         % Variance sub bias
    BiasNoiseVariance = BiasVariance * (1-exp(-2/tau)); % Variance sub nk
    BiasNoiseSigma = sqrt(BiasNoiseVariance);
    BiasNoiseMean = 0;                                  % 
    
    bias1 = zeros(length(Acc), 1); % b sub k
    bias2 = zeros(length(Acc), 1);
    bias3 = zeros(length(Acc), 1);
    for t = 1:length(Acc)
        BiasNoise1 = normrnd(BiasNoiseMean,BiasNoiseSigma);
        BiasNoise2 = normrnd(BiasNoiseMean,BiasNoiseSigma);
        BiasNoise3 = normrnd(BiasNoiseMean,BiasNoiseSigma);
        sensorNoise1 = normrnd(SensorNoiseMean,SensorNoiseSigma);
        sensorNoise2 = normrnd(SensorNoiseMean,SensorNoiseSigma);
        sensorNoise3 = normrnd(SensorNoiseMean,SensorNoiseSigma);
        
        if t > 1
            % Calculate bias noise - n sub k
            bias1(t,:) = c * bias1(t-1,:) + BiasNoise1;
            bias2(t,:) = c * bias2(t-1,:) + BiasNoise2;
            bias3(t,:) = c * bias3(t-1,:) + BiasNoise3;
        else
            bias1(t,:) = BiasNoise1;
            bias2(t,:) = BiasNoise2;
            bias3(t,:) = BiasNoise3;
        end
        
        Acc(t,1) = Acc(t,1) + bias1(t,:) + sensorNoise1;
        Acc(t,2) = Acc(t,2) + bias2(t,:) + sensorNoise2;
        Acc(t,3) = Acc(t,3) + bias3(t,:) + sensorNoise3;
        
    end
    
    disp(bias1(1:100,:));


    %measurementError = bias + SensorNoise;
   % Acc = Acc + measurementError;
end

