function [ ] = change5( filename, source_dir, report_dir, PSD_plot_type, sample_rate )
% 9DOF Change Evaluation 
% filename = root file name for a file containing an Nx3 matrix of sample data
% source_dir = filename where source data is to be found
% report_dir = directory in which to place output files
%   Input parameter is a string filename (for the IMU data file).
% PSD_plot_type = 0 = SKIP PSD
% PSD_plot_type = 1 for linear Y
% PSD_plot_type = 2 for log Y
% sample_rate is in Hertz
clc;                % Clear the command window
close all;          % Clear all figures

if (~ exist(report_dir, 'dir'))
    fprintf('Requested destination directory (%s) does not exist.  Creating it.\n', report_dir);
    mkdir(report_dir)
end
data=load(strcat(source_dir,'/',filename, '.dat'));
[num_points, num_d] = size(data);
% For original data, the first column in the data is simply a integer sample number.  Convert to
% time by dividing by the sample rate.  Resampled input data may have
% fractional numbers, but scaling should work the same.
time = data(:,1)/sample_rate; % samples are provided sample_rate times per second
%figure('Position', [10 10 810 410]);

% Plot Acceleration
subdata = data(:,5:7)*2.93; % Scale factor is approximate mg/LSB for the IMU sensor
units = 'milli-gs';
plot(time, subdata(:,1), 'r', time, subdata(:,2), 'k', time, subdata(:,3), 'b', 'LineWidth', 2);
set(gca, 'LineWidth', 2); 
grid on;
str = sprintf('%s Acceleration vs Time', filename);
title(str,'FontSize', 18, 'Interpreter', 'none');
xlabel('Time in Seconds', 'FontSize', 14);
ylabel('Acceleration in mg', 'FontSize', 14);
legend('X', 'Y', 'Z');
str = sprintf('Data Set: %s', filename);
y = min(min(subdata));
text(0,y, str, 'FontSize', 10, 'VerticalAlignment', 'Bottom', 'Interpreter', 'none');
fn=strcat(report_dir,'/', filename, '_acc.jpg')
print ('-djpeg', fn);
close all;

% Plot PSD of acceleration

if (plot_PSD3(strcat(filename,' Acceleration'), sample_rate, subdata, 'Acc^2 in mg^2', PSD_plot_type))
    fn=strcat(report_dir,'/', filename, '_acc_PSD.jpg')
    print ('-djpeg', fn)
end
close all;
    acc_result.Variable = 'Value';
    acc_result.filename = filename;
    acc_result.num_points = num_points;
    acc_result.final_time = max(time);
    acc_result.minx = min(subdata(:,1));
    acc_result.maxx = max(subdata(:,1));
    acc_result.miny = min(subdata(:,2));
    acc_result.maxy = max(subdata(:,2));
    acc_result.minz = min(subdata(:,3));
    acc_result.maxz = max(subdata(:,3));
    acc_result.rangex = abs(acc_result.maxx - acc_result.minx);
    acc_result.rangey = abs(acc_result.maxy - acc_result.miny);
    acc_result.rangez = abs(acc_result.maxz - acc_result.minz);
    acc_result.units = units;

% Plot Magnetics
subdata = data(:,2:4)*.0488;  % Scale factor is approximate uT/LSB for the IMU sensor
units = 'microteslas';
total_field = 48.3218; % Earth magnetic field in microtelsas
plot(time, subdata(:,1), 'r', time, subdata(:,2), 'k', time, subdata(:,3), 'b', 'LineWidth', 2);
set(gca, 'LineWidth', 2); 
grid on;
str = sprintf('%s Magnetic Field vs Time', filename);
title(str,'FontSize', 18, 'Interpreter', 'none');
xlabel('Time in Seconds', 'FontSize', 14);
ylabel('Magnetic Field in uTeslas', 'FontSize', 14);
legend('X', 'Y', 'Z');
str = sprintf('Data Set: %s', filename);
y=min(min(subdata));
text(0,y, str, 'FontSize', 10, 'VerticalAlignment', 'Bottom', 'Interpreter', 'none');
fn=strcat(report_dir,'/', filename, '_mag.jpg')
print ('-djpeg', fn)
close all;

% Plot PSD of Magnetics
if (plot_PSD3(strcat(filename,' Magnetics'), sample_rate, subdata, 'H^2 in uTeslas^2', PSD_plot_type))
    fn=strcat(report_dir,'/', filename, '_mag_PSD.jpg')
    print ('-djpeg', fn)
end
close all;
    mag_result.Variable = 'Value';
    mag_result.filename = filename;
    mag_result.num_points = num_points;
    mag_result.final_time = max(time);
    mag_result.minx = min(subdata(:,1));
    mag_result.maxx = max(subdata(:,1));
    mag_result.miny = min(subdata(:,2));
    mag_result.maxy = max(subdata(:,2));
    mag_result.minz = min(subdata(:,3));
    mag_result.maxz = max(subdata(:,3));
    mag_result.rangex = abs(mag_result.maxx - mag_result.minx);
    mag_result.rangey = abs(mag_result.maxy - mag_result.miny);
    mag_result.rangez = abs(mag_result.maxz - mag_result.minz);
    mag_result.units = units;

% Plot Angular Velocity
subdata = data(:,8:10)*0.98; % Scale factor is approximate degrees/second for IMU sensor
units = 'degrees/second';
plot(time, subdata(:,1), 'r', time, subdata(:,2), 'k', time, subdata(:,3), 'b', 'LineWidth', 2);
set(gca, 'LineWidth', 2); 
grid on;
str = sprintf('%s Angular Velocity vs Time', filename);
title(str,'FontSize', 18, 'Interpreter', 'none');
xlabel('Time in Seconds', 'FontSize', 14);
ylabel('Angular Velocity in degrees/sec', 'FontSize', 14);
legend('X', 'Y', 'Z');
str = sprintf('Data Set: %s', filename);
y=min(min(subdata));
text(0,y, str, 'FontSize', 10, 'VerticalAlignment', 'Bottom', 'Interpreter', 'none');
fn=strcat(report_dir,'/', filename, '_av.jpg')
print ('-djpeg', fn)
close all;

% Plot PSD of Angular Velocity
if (plot_PSD3(strcat(filename,' Angular Velocity'), sample_rate, subdata, 'Angular Velocity^2 in (\circ/sec)^2', PSD_plot_type))
    fn=strcat(report_dir,'\', filename, '_av_PSD.jpg')
    print ('-djpeg', fn)
end
close all;

    av_result.Variable = 'Value';
    av_result.filename = filename;
    av_result.num_points = num_points;
    av_result.final_time = max(time);
    av_result.minx = min(subdata(:,1));
    av_result.maxx = max(subdata(:,1));
    av_result.miny = min(subdata(:,2));
    av_result.maxy = max(subdata(:,2));
    av_result.minz = min(subdata(:,3));
    av_result.maxz = max(subdata(:,3));
    av_result.rangex = abs(av_result.maxx - av_result.minx);
    av_result.rangey = abs(av_result.maxy - av_result.miny);
    av_result.rangez = abs(av_result.maxz - av_result.minz);
    av_result.units = units;
 
fn=strcat(report_dir,'/', filename, '_summary.html')
fid = fopen(fn, 'w');
%fprintf(fid, '<HTML><HEAD><TITLE>%s Summary</TITLE>', filename);
%fprintf(fid, '<link rel=\"stylesheet\" title=\"Report Styles\" type=\"text/css\" href=\"style.css\">');
%fprintf(fid, '</HEAD><BODY>\n');
%fprintf(fid, '<BODY>\n');
fprintf(fid, '<TABLE>\n');
fprintf(fid, '<CAPTION>%s Sensor Statistics</CAPTION>\n', filename);
fprintf(fid, '<THEAD>\n');
fprintf(fid, '<TR><TD>Quantity</TD><TD>Acceleration</TD><TD>Magnetics</TD><TD>Angular Velocity</TD></TR>');
fprintf(fid, '</THEAD>\n');
fprintf(fid, '<TR><TD>Units</TD><TD>%s</TD><TD>%s</TD><TD>%s</TD></TR>', acc_result.units, mag_result.units, av_result.units );
fprintf(fid, '<TR><TD>NumPoints</TD><TD>%d</TD><TD>%d</TD><TD>%d</TD></TR>', acc_result.num_points, mag_result.num_points, av_result.num_points );
fprintf(fid, '<TR><TD>MaxTime</TD><TD>%10.2f</TD><TD>%10.2f</TD><TD>%10.2f</TD></TR>', acc_result.final_time, mag_result.final_time, av_result.final_time );
fprintf(fid, '<TR class=\"bot\"><TD>MaxX</TD><TD>%10.2f</TD><TD>%10.2f</TD><TD>%10.2f</TD></TR>', acc_result.maxx, mag_result.maxx, av_result.maxx );
fprintf(fid, '<TR class=\"bot\"><TD>MinX</TD><TD>%10.2f</TD><TD>%10.2f</TD><TD>%10.2f</TD></TR>', acc_result.minx, mag_result.minx, av_result.minx );
fprintf(fid, '<TR class=\"bot\"><TD>RangeX</TD><TD>%10.2f</TD><TD>%10.2f</TD><TD>%10.2f</TD></TR>', acc_result.rangex, mag_result.rangex, av_result.rangex );
fprintf(fid, '<TR><TD>MaxY</TD><TD>%10.2f</TD><TD>%10.2f</TD><TD>%10.2f</TD></TR>', acc_result.maxy, mag_result.maxy, av_result.maxy );
fprintf(fid, '<TR><TD>MinY</TD><TD>%10.2f</TD><TD>%10.2f</TD><TD>%10.2f</TD></TR>', acc_result.miny, mag_result.miny, av_result.miny );
fprintf(fid, '<TR><TD class>RangeY</TD><TD>%10.2f</TD><TD>%10.2f</TD><TD>%10.2f</TD></TR>', acc_result.rangey, mag_result.rangey, av_result.rangey );
fprintf(fid, '<TR class=\"bot\"><TD>MaxZ</TD><TD>%10.2f</TD><TD>%10.2f</TD><TD>%10.2f</TD></TR>', acc_result.maxz, mag_result.maxz, av_result.maxz );
fprintf(fid, '<TR class=\"bot\"><TD>MinZ</TD><TD>%10.2f</TD><TD>%10.2f</TD><TD>%10.2f</TD></TR>', acc_result.minz, mag_result.minz, av_result.minz );
fprintf(fid, '<TR class=\"bot\"><TD>RangeZ</TD><TD>%10.2f</TD><TD>%10.2f</TD><TD>%10.2f</TD></TR>', acc_result.rangez, mag_result.rangez, av_result.rangez );
fprintf(fid, '</TABLE>\n');
%fprintf(fid, '</BODY></HTML>\n');

fclose(fid);

end

