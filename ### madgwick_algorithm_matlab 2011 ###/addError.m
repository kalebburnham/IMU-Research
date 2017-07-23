function [Acc,Gyr] = addError(Acc,Gyr)
    
    % Sensor 1 = Accelerometer
    % Sensor 2 = Gyroscope

    for sensor = 1:2
        if sensor == 1 % Acceleration Parameters
            tau = 1;    % Correlation Coefficient
            SensorNoiseStdDev = 0.012;
            SensorNoiseMean = 0;
            BiasStability = 0.04;
            BiasNoiseMean = 0;
        else % Gyroscope Parameters
            tau = 1;
            SensorNoiseMean = 0;
            SensorNoiseStdDev = 0.0087;
            BiasNoiseMean = 0;
            BiasStability = 0.015;
        end
        
        % Set variables from the formulas
        c = exp(-1/tau);                                    % constant defined by Gauss-Markov model
        BiasStdDev = BiasStability/sqrt(tau);               % Sigma sub bias
        BiasVariance = BiasStdDev^2;                        % Variance sub bias
        BiasNoiseVariance = BiasVariance * (1-exp(-2/tau)); % Variance sub nk
        BiasNoiseStdDev = sqrt(BiasNoiseVariance);

        % Compute bias and sensor noise as a random walk
        bias = zeros(length(Acc), 3); % b sub k
        SensorNoise = zeros(length(Acc), 3);
        for t = 1:length(Acc)
            BiasNoise = normrnd(BiasNoiseMean, BiasNoiseStdDev, [1 3]);
            SensorNoise(t,:) = normrnd(SensorNoiseMean, SensorNoiseStdDev, [1 3]);

            if t > 1
                bias(t,:) = c * bias(t-1,:) + BiasNoise;
            else
                bias(t,:) = BiasNoise; % Only on first iteration
            end

        end
        
        % Compute total measurement error
        measurementError = bias + SensorNoise;
        
        % Add error to sensor
        if sensor == 1
            Acc = Acc + measurementError;
        else
            Gyr = Gyr + measurementError;
        end
    end
    
    
    
end

