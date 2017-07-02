function [  ] = run( filename, report_dir, PSD_plot_type)
% Simone - run('Nudge', 'report', 1);
%run will generate tables and figures for all IMU samples, which must be
%explicity listed in the change4 calls below.
report_dir1 = strcat(report_dir,'/',filename,'/unfiltered');
change5(filename, 'data', report_dir1, PSD_plot_type, 125);
end

