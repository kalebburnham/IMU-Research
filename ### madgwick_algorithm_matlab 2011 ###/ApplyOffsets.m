% Simulates an IMU at a specified position relative to the origin and an
% orientation specified by DCM. Rot is the derivative of
% Gyr and must be in radians.

function [Acc,Gyr,Mag] = ApplyOffsets(Position, DCM, Rot, Acc, Gyr, Mag)

% (1) Check that all the inputs and sizes of inputs are proper.
% use isequal - size is a vector
% TODO

if ~isequal(size(Gyr),size(Rot))
    error('Gyr and Rot are not the same length');
end

% (2) Normalize position vector
% Put in conditional so this function can be tested with no position
% changes
% Puts position at a distance of 1. I think the unit is 1 radian, but not
% sure. Acceleration would change based on the distance of a radian??
if norm(Position) > 0
    Position = Position / norm(Position);
end

% (3) Alter accelerations based on acceleration in each axis based on
% rotations around global axes.
% (a) Rotation around global x axis
for t = 1:length(Acc)
    y = Position(1,2);
    z = Position(1,3);
    magnitude = sqrt(y^2 + z^2);
    rot = Rot(t,1);
    
    if (norm(y) > 0)
        Acc(t,3) = Acc(t,3) + sqrt(magnitude - z^2) * (y/norm(y)) * rot;
    end
    if (norm(z) > 0) % Prevents divide by zero
        Acc(t,2) = Acc(t,2) + sqrt(magnitude - y^2) * (z/norm(z)) * rot;
    end
end
    

% (b) Rotation around global y axis
for t = 1:length(Acc)
    x = Position(1,1);
    z = Position(1,3);
    magnitude = sqrt(x^2 + z^2);
    rot = Rot(t,2);
    
    if (norm(z) > 0)
        Acc(t,1) = Acc(t,1) + sqrt(magnitude - x^2) * (z/norm(z)) * rot;
    end
    if (norm(x) > 0)
        Acc(t,3) = Acc(t,3) + sqrt(magnitude - z^2) * (x/norm(x)) * rot;
    end
end

% (c) Rotation around global z axis
for t = 1:length(Acc)
    x = Position(1,1);
    y = Position(1,2);
    magnitude = sqrt(x^2 + y^2);
    rot = Rot(t,3);
    
    if (norm(y) > 0)
        Acc(t,1) = Acc(t,1) + sqrt(magnitude - x^2) * (-y/norm(y)) * rot; % -y is intentional
    end
    if (norm(x) > 0)
        Acc(t,2) = Acc(t,2) + sqrt(magnitude - y^2) * (x/norm(x)) * rot;
    end
end

Acc = Acc * DCM;
Gyr = Gyr * DCM;
Mag = Mag * DCM;

end

