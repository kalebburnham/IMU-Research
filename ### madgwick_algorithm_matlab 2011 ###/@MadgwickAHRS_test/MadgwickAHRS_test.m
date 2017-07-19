classdef MadgwickAHRS_test < handle
%MADGWICKAHRS Implementation of Madgwick's IMU and AHRS algorithms
%
%
%   Date          Author          Notes
%   28/09/2011    SOH Madgwick    Initial release

    %% Public properties
    properties (Access = public)
        SamplePeriod = 1/100;
        Quaternion = [1 0 0 0];     % output quaternion describing the Earth relative to the sensor
        Beta = 1;               	% algorithm gain
    end

    %% Public methods
    methods (Access = public)
        function obj = MadgwickAHRS(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SamplePeriod'), obj.SamplePeriod = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Quaternion'), obj.Quaternion = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Beta'), obj.Beta = varargin{i+1};
                else error('Invalid argument');
                end
            end
        end

        function obj = Update(obj, Gyroscope, Accelerometer, Magnetometer)
            q = obj.Quaternion; % short name local variable for readability

            gx = Gyroscope(1);
            gy = Gyroscope(2);
            gz = Gyroscope(3);
            ax = Accelerometer(1);
            ay = Accelerometer(2);
            az = Accelerometer(3);
            mx = Magnetometer(1);
            my = Magnetometer(2);
            mz = Magnetometer(3);
            q0 = obj.Quaternion(1);
            q1 = obj.Quaternion(2);
            q2 = obj.Quaternion(3);
            q3 = obj.Quaternion(4);

            qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
          	qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
          	qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
          	qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

            if(norm(Accelerometer) == 0), return; end	% handle NaN
                
            % Normalise accelerometer measurement
            recipNorm = 1/sqrt(ax * ax + ay * ay + az * az);
            ax = ax * recipNorm;
            ay = ay * recipNorm;
            az = az * recipNorm;

            % Normalise magnetometer measurement
            recipNorm = 1/sqrt(mx * mx + my * my + mz * mz);
            mx = mx *recipNorm;
            my = my * recipNorm;
            mz = mz * recipNorm;

            % Auxiliary variables to avoid repeated arithmetic
            a2q0mx = 2.0 * q0 * mx;
            a2q0my = 2.0 * q0 * my;
            a2q0mz = 2.0 * q0 * mz;
            a2q1mx = 2.0 * q1 * mx;
            a2q0 = 2.0 * q0;
            a2q1 = 2.0 * q1;
            a2q2 = 2.0 * q2;
            a2q3 = 2.0 * q3;
            a2q0q2 = 2.0 * q0 * q2;
            a2q2q3 = 2.0 * q2 * q3;
            q0q0 = q0 * q0;
            q0q1 = q0 * q1;
            q0q2 = q0 * q2;
            q0q3 = q0 * q3;
            q1q1 = q1 * q1;
            q1q2 = q1 * q2;
            q1q3 = q1 * q3;
            q2q2 = q2 * q2;
            q2q3 = q2 * q3;
            q3q3 = q3 * q3;

            % Reference direction of Earth's magnetic field
            hx = mx * q0q0 - a2q0my * q3 + a2q0mz * q2 + mx * q1q1 + a2q1 * my * q2 + a2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
            hy = a2q0mx * q3 + my * q0q0 - a2q0mz * q1 + a2q1mx * q2 - my * q1q1 + my * q2q2 + a2q2 * mz * q3 - my * q3q3;
            a2bx = sqrt(hx * hx + hy * hy);
            a2bz = -a2q0mx * q2 + a2q0my * q1 + mz * q0q0 + a2q1mx * q3 - mz * q1q1 + a2q2 * my * q3 - mz * q2q2 + mz * q3q3;
            a4bx = 2.0 * a2bx;
            a4bz = 2.0 * a2bz;

            % Gradient decent algorithm corrective step
            s0 = -a2q2 * (2.0 * q1q3 - a2q0q2 - ax) + a2q1 * (2.0 * q0q1 + a2q2q3 - ay) - a2bz * q2 * (a2bx * (0.5 - q2q2 - q3q3) + a2bz * (q1q3 - q0q2) - mx) + (-a2bx * q3 + a2bz * q1) * (a2bx * (q1q2 - q0q3) + a2bz * (q0q1 + q2q3) - my) + a2bx * q2 * (a2bx * (q0q2 + q1q3) + a2bz * (0.5 - q1q1 - q2q2) - mz);
            s1 = a2q3 * (2.0 * q1q3 - a2q0q2 - ax) + a2q0 * (2.0 * q0q1 + a2q2q3 - ay) - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + a2bz * q3 * (a2bx * (0.5 - q2q2 - q3q3) + a2bz * (q1q3 - q0q2) - mx) + (a2bx * q2 + a2bz * q0) * (a2bx * (q1q2 - q0q3) + a2bz * (q0q1 + q2q3) - my) + (a2bx * q3 - a4bz * q1) * (a2bx * (q0q2 + q1q3) + a2bz * (0.5 - q1q1 - q2q2) - mz);
            s2 = -a2q0 * (2.0 * q1q3 - a2q0q2 - ax) + a2q3 * (2.0 * q0q1 + a2q2q3 - ay) - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-a4bx * q2 - a2bz * q0) * (a2bx * (0.5 - q2q2 - q3q3) + a2bz * (q1q3 - q0q2) - mx) + (a2bx * q1 + a2bz * q3) * (a2bx * (q1q2 - q0q3) + a2bz * (q0q1 + q2q3) - my) + (a2bx * q0 - a4bz * q2) * (a2bx * (q0q2 + q1q3) + a2bz * (0.5 - q1q1 - q2q2) - mz);
            s3 = a2q1 * (2.0 * q1q3 - a2q0q2 - ax) + a2q2 * (2.0 * q0q1 + a2q2q3 - ay) + (-a4bx * q3 + a2bz * q1) * (a2bx * (0.5 - q2q2 - q3q3) + a2bz * (q1q3 - q0q2) - mx) + (-a2bx * q0 + a2bz * q2) * (a2bx * (q1q2 - q0q3) + a2bz * (q0q1 + q2q3) - my) + a2bx * q1 * (a2bx * (q0q2 + q1q3) + a2bz * (0.5 - q1q1 - q2q2) - mz);
            recipNorm = 1/sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); % normalise step magnitude
            s0 = s0 * recipNorm;
            s1 = s1 * recipNorm;
            s2 = s2 * recipNorm;
            s3 = s3 * recipNorm;

            % Apply feedback step
            qDot1 = qDot1 - obj.Beta * s0;
            qDot2 = qDot2 - obj.Beta * s1;
            qDot3 = qDot3 - obj.Beta * s2;
            qDot4 = qDot4 - obj.Beta * s3;
          	

            % Integrate rate of change of quaternion to yield quaternion
          	q0 = q0 + qDot1 * (1.0 / sampleFreq);
          	q1 = q1 + qDot2 * (1.0 / sampleFreq);
          	q2 = q2 + qDot3 * (1.0 / sampleFreq);
          	q3 = q3 + qDot4 * (1.0 / sampleFreq);

            % Normalise quaternion
          	recipNorm = 1/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
          	q0 = q0 * recipNorm;
          	q1 = q1 * recipNorm;
          	q2 = q2 * recipNorm;
          	q3 = q3 * recipNorm;

            q = [q0 q1 q2 q3];
            obj.Quaternion = q / norm(q);
        end
    end
end
