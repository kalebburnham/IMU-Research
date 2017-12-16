classdef BasicAHRS < handle
    % BASIC AHRS - Weighted method based on Diaz et al. 2015
    %   Integration of two basic AHRS blocks
    
    properties (Access = public)
        Quaternion = [1 0 0 0];
        Gravity = 9.8;
        SamplePeriod = 1/100;
        Gamma = 0.995; 
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

            %......................Quat from Acc and Mag....................
            % Create acc estimation in euler angles
            AccEuler(1) = atan2(Accelerometer(2), Accelerometer(3)); %Roll (exact solution but unreliable close to +/-90º)
            AccEuler(2) = atan2(-Accelerometer(1),sqrt(Accelerometer(2)^2 + Accelerometer(3)^2)); %Pitch
                  
            % Create rotation matrices to rotate magnetometer readings to
            % a flat plane and calculate yaw
            Rx = [1 0 0; 0 cos(AccEuler(1)) sin(AccEuler(1)); 0 -sin(AccEuler(1)) cos(AccEuler(1))]; % Rotates along x axis
            Ry = [cos(AccEuler(2)) 0 -sin(AccEuler(2)); 0 1 0; sin(AccEuler(2)) 0 cos(AccEuler(2))]; % Rotates along y axis
            Magnetometer = Magnetometer * Rx * Ry;
            AccEuler(3) = atan2(-Magnetometer(2),Magnetometer(1));
            
           
           % Convert from Euler to Quaternion  (not the same using Robotics eul2quat function):
           % AccMagQuat = eul2quat(AccEuler); % 'ZYX'
            rotm = euler2rotMat(AccEuler(1), AccEuler(2), AccEuler(3));
            AccMagQuat = rotMat2quatern(rotm');
            AccMagQuat=AccMagQuat/norm(AccMagQuat);
             
             
            %......................Quat from Gyr....................
            % Gyroscope takes the previous estimation and updates based
            % on the integration of the sensor data
            w_norm = norm(Gyroscope);
            if w_norm > 0
                q_w = [cos(w_norm/2*obj.SamplePeriod), sin(w_norm/2*obj.SamplePeriod)*Gyroscope(1)/w_norm, sin(w_norm/2*obj.SamplePeriod)*Gyroscope(2)/w_norm, sin(w_norm/2*obj.SamplePeriod)*Gyroscope(3)/w_norm];
                obj.Quaternion = quaternProd(obj.Quaternion,q_w);
            else
                %obj.Quaternion = obj.Quaternion; % There was no change so quit
            end
            obj.Quaternion=obj.Quaternion/norm(obj.Quaternion); % normalize
            
            %.......................Fusion....................
            % Fused the estimations of euler angles from Acc/Mag and Gyr
     
            obj.Quaternion= obj.Gamma * obj.Quaternion + (1-obj.Gamma) * AccMagQuat;
            
            
        end
    end
end

