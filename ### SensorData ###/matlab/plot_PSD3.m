function [ sts ] = plot_PSD3( ID_text, Fs, data, quantity, plot_type )
%plot_3Hfft(Fs, data, quantity) performs PSD of X/Y/Z data and plots it in 3X1 format
% FS = Sampling Frequency
% data = signal of interest
% quantity = string describing variable being analyzed
% plot_type = 0 = SKIP (just do return)
% plot_type = 1 for linear Y
% plot_type = 2 for log Y

if (plot_type==0)
    sts=0;
    return;
end

[num_points, num_d] = size(data);
data = data - repmat(mean(data), num_points, 1); % Remove the DC component to improve analysis.

[PSDX, fx] = pwelch(data(:,1));
[PSDY, fy] = pwelch(data(:,2));
[PSDZ, fz] = pwelch(data(:,3));
% Scale results from radians/sample to Hertz by multiplying by 125
% samples/sec times 1/(2pi)
fx = fx * Fs / (2*pi);
fy = fy * Fs / (2*pi);
fz = fz * Fs / (2*pi);
PSDX = PSDX * Fs / (2*pi);
PSDY = PSDY * Fs / (2*pi);
PDSZ = PSDZ * Fs / (2*pi);

figure('Position', [10 10 510 810]);
subplot(3,1,1);
if (plot_type==1)
    plot(fx, PSDX, 'r', 'LineWidth', 2);
    scale = '(linear)';
else
    % plot_type == 2
    semilogy(fx, PSDX, 'r', 'LineWidth', 2);
    scale = '(log)';
end    
set(gca, 'LineWidth', 2); 
set(gca, 'XTick', 0:5:65)
grid on;
str = sprintf('%s X-Axis %s PSD', ID_text, scale);
title(str, 'FontSize', 18, 'Interpreter', 'none');
xlabel('Frequency (Hz)', 'FontSize', 12)
ylabel(quantity, 'FontSize', 10)

subplot(3,1,2);
if (plot_type==1)
    plot(fy, PSDY, 'k', 'LineWidth', 2);
    scale = '(linear)';
else
    % plot_type == 2
    semilogy(fy, PSDY, 'k', 'LineWidth', 2);
    scale = '(log)';
end
set(gca, 'LineWidth', 2); 
set(gca, 'XTick', 0:5:65)
grid on;
str = sprintf('%s Y-Axis %s PSD', ID_text, scale);
title(str, 'FontSize', 18, 'Interpreter', 'none');
xlabel('Frequency (Hz)', 'FontSize', 12)
ylabel(quantity, 'FontSize', 10)

subplot(3,1,3);
if (plot_type==1)
    plot(fz, PSDZ, 'b', 'LineWidth', 2);
    scale = '(linear)';
else
    % plot_type == 2
    semilogy(fz, PSDZ, 'b', 'LineWidth', 2);
    scale = '(log)';
end
set(gca, 'LineWidth', 2); 
set(gca, 'XTick', 0:5:65)
grid on;
str = sprintf('%s Z-Axis %s PSD', ID_text, scale);
title(str, 'FontSize', 18, 'Interpreter', 'none');
xlabel('Frequency (Hz)', 'FontSize', 12)
ylabel(quantity, 'FontSize', 10)
sts=1;
end

