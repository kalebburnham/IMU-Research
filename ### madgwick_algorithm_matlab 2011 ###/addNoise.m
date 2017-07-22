function [Acc, Gyr] = addNoise(Acc, Gyr)
%Add noise and bias


% Noise
mean = 0;
AccSigma = 0.012; % Std Dev.
GyrSigma = 0.0087;

AccNoiseRandomWalk = zeros(size(Acc));
GyrNoiseRandomWalk = zeros(size(Gyr));

for t = 2:length(Acc)
    if t > 1
        AccNoiseRandomWalk(t,:) = AccNoiseRandomWalk(t-1,:) + normrnd(mean,AccSigma,[1,3]);
        GyrNoiseRandomWalk(t,:) = GyrNoiseRandomWalk(t-1,:) + normrnd(mean,GyrSigma,[1,3]);
    else 
        AccNoiseRandomWalk(1,:) = normrnd(mean,AccSigma,[1 3]);
        GyrNoiseRandomWalk(1,:) = normrnd(mean,GyrSigma,[1 3]);
    end
end

Acc = Acc + AccNoiseRandomWalk;
Gyr = Gyr + GyrNoiseRandomWalk;

figure;
hold on;
plot(1:length(Acc), GyrNoiseRandomWalk(:,1), 'r');
hold off;


end

