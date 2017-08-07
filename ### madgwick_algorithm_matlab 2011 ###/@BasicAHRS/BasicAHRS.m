classdef BasicAHRS < handle
    % BASIC AHRS - Weighted method based on Diaz et al. 2015
    %   Integration of two basic AHRS blocks
    
    properties (Access = public)
        Quaternion = [1 0 0 0];
        Gravity = 9.786;
        SamplePeriod = 1/100;
        Gamma = 0.5; 
    end
    
    methods (Access = public)
        function obj = BasicAHRS(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SamplePeriod'), obj.SamplePeriod = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Gamma'), obj.Gamma = varargin{i+1};
                elseif  strcmp (varargin{i}, 'Gravity'), obj.Gravity = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Quaternion'), obj.Quaternion = varargin{i+1};
                else error('Invalid argument');
                end 
            end
        end
        
        function obj = Update(obj, Gyroscope, Accelerometer, Magnetometer)
            AccEuler = zeros(1,3);

            % Normalise accelerometer measurement
            if(norm(Accelerometer) == 0), return; end	% handle NaN
            Accelerometer = Accelerometer / norm(Accelerometer);	% normalise

            % Normalise magnetometer measurement
            if(norm(Magnetometer) == 0), return; end	% handle NaN
            Magnetometer = Magnetometer / norm(Magnetometer);	% normalise	

            % Create acc estimation in euler angles
            AccEuler(1) = atan(Accelerometer(2) / Accelerometer(3));
            AccEuler(2) = atan(-Accelerometer(1) / sqrt(Accelerometer(2)^2 + Accelerometer(3)^2));
                  
            % Create rotation matrices to rotate magnetometer readings to
            % a flat plane and calculate yaw
            Rx = [1 0 0; 0 cos(AccEuler(1)) sin(AccEuler(1)); 0 -sin(AccEuler(1)) cos(AccEuler(1))]; % Rotates along x axis
            Ry = [cos(AccEuler(2)) 0 -sin(AccEuler(2)); 0 1 0; sin(AccEuler(2)) 0 cos(AccEuler(2))]; % Rotates along y axis
            Magnetometer = Magnetometer * Rx * Ry;
            AccEuler(3) = atan(-Magnetometer(1) / Magnetometer(2));
            declination = 0.76; % Madrid = 0.76
            isNeg = (AccEuler(:,3) < 0);
            if (isNeg == 1)
                AccEuler(:,3) = AccEuler(:,3) + declination; %(2*pi/180);
            else
                AccEuler(:,3) = AccEuler(:,3) - declination; %(2*pi/180);
            end
            
            % Gyroscope takes the previous estimation and updates based
            % on the integration of the sensor data
            w_norm = norm(Gyroscope);
            if w_norm > 0
                q_w = [cos(w_norm/2*obj.SamplePeriod), sin(w_norm/2*obj.SamplePeriod)*Gyroscope(1)/w_norm, sin(w_norm/2*obj.SamplePeriod)*Gyroscope(2)/w_norm, sin(w_norm/2*obj.SamplePeriod)*Gyroscope(3)/w_norm];
                GyrQuat = quaternProd(obj.Quaternion,q_w);
            else
                GyrQuat = obj.Quaternion; % There was no change so quit
            end
            AccEuler_saved = AccEuler(3);
            AccEuler(3) = AccEuler(1);
            AccEuler(1) = AccEuler_saved;
            AccQuat = euler2quatern(AccEuler); % 'ZYX' - From robotics toolkit
            % Fused the estimations of euler angles from Acc/Mag and Gyr
            FusedQuat = obj.Gamma * GyrQuat + (1-obj.Gamma) * AccQuat;
            
            % Set quaternion
            obj.Quaternion = FusedQuat; %rotMat2quatern(euler2rotMat(FusedEuler(1), FusedEuler(2), FusedEuler(3)));
            
        end
    end
end
