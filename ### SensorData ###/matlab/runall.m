function [  ] = runall( input_args )
%runall will generate tables and figures for all IMU samples, which must be
%explicity listed in the change4 calls below.
%   No input parameters required.
run('laptop_tip4', 'report', 1);
run('laptop_tip3', 'report', 1);
run('laptop_tip2', 'report', 1);
run('laptop_tip1', 'report', 1);
run('pedometry1', 'report', 1);
run('pedometry2', 'report', 1);
run('pedometry3', 'report', 1);
run('pedometry4', 'report', 1);
run('pedometry5', 'report', 1);
run('angled_rotate', 'report', 2);
run('sound_test7', 'report', 2);
run('sound_test6', 'report', 2);
run('gyro_orientation_test', 'report', 2);
run('sound_test5', 'report', 2);
run('sound_test4', 'report', 2);
run('sound_test3', 'report', 2);
run('sound_test2', 'report', 2);
run('gyro_sensitivity_to_sound1', 'report', 2);
run('sound_baseline', 'report', 2);
run('Mixed_Driving', 'report', 2);
run('StraightFreeway', 'report', 2);
run('Northbound_101', 'report', 2);
run('CityDrivingToGarage1', 'report', 2);
run('FallFromDesk1', 'report', 1);
run('FallFromDesk2', 'report', 1);
run('FallFromDesk3', 'report', 1);
run('FallFromDesk4', 'report', 1);
run('FallFromDesk5', 'report', 1);
run('button_click', 'report', 2);
run('double_tap1', 'report', 2);
run('double_tap2', 'report', 2);
run('FastPickup', 'report', 1);
run('freefall', 'report', 1);
run('freefall2', 'report', 1);
run('Nudge', 'report', 1);
run('Pickup', 'report', 2);
run('pocketed1', 'report', 2);
run('shake1', 'report', 1);
run('single_tap1', 'report', 2);
run('single_tap2', 'report', 2);
run('Slide', 'report', 1);
run('SlowPickup', 'report', 2);
run('stationary', 'report', 2);
run('manual_study', 'report', 2);
end

