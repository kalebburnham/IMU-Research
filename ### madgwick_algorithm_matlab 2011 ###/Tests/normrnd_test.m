values = zeros(10000, 2);

sigma = 0.012;
mean = 0;

for t = 1:10000
    n = normrnd(mean, sigma);
    values(t,:) = [n/sigma, n];
end

figure;
hold on;
scatter(values(:,1), values(:,2));