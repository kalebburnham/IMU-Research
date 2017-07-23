function [Acc,Gyr] = addError(Acc,Gyr)
    % Acceleration First
    
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
    
    bias = zeros(length(Acc), 3); % b sub k
    
    for t = 1:length(Acc)
        BiasNoise = normrnd(BiasNoiseMean, BiasNoiseSigma, [1 3]);
        SensorNoise = normrnd(SensorNoiseMean, SensorNoiseSigma, [1 3]);
        
        if t > 1
            % Calculate bias, b sub k
            bias(t,:) = c * bias(t-1,:) + BiasNoise;
        else
            bias(t,:) = BiasNoise;
        end
    end

    measurementError = bias + SensorNoise;
    Acc = Acc + measurementError;
    
    % Gyrometer next
end

