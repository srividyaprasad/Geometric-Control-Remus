% REMUSSIM.M Vehicle Simulator
% M-FILE INPUTS
% + coeffs.mat - generated by COEFFS.M, typically for each run
% + vdata.mat - generated by COEFFS.M, typically for each run
clear ; % clear all variables
clc ;
fprintf("\n\n REMUS DYNAMICS SIMULATOR\n");
fprintf ('Timothy Prestero, MIT/WHOI\n\n\n');
%%%%%%%% load vehicle_type ;
fprintf('NOTE: Model using %s REMUS dimensions.\n\n\n', vehicle);
% Check coeffs, initial conditions, and control input vector
choose_setup = input(' Run set-up (y/n):','s'); 
if choose_setup == 'y'
sim_setup;
else
showall;
end
%
% Output flags
%
show_step = 1 ; show_speed = 0 ; show_pos = 0;
run_savedata = 0 ; run_plots = 0 ; choose_int = 0;
% choosesetup = 1 ;
% check working directory
cd outputs ;
% create .mat files
d = clock ; 
yy = d(1) ; mo = d(2) ; dd = d(3) ; 
hh = d(4) ; mm = d(5) ; ss = d(6) ; 
date_string = datestr(datenum(yy,mo,dd),1);
time_string = datestr(datenum(yy,mo,dd,hh,mm,ss),13);
%% generate random filename
% [dummy, file_string, dummy, dummy] = fileparts(tempname)
% disp(sprintf('\nCurrent simulator data files:'));
% ls *.mat;
% file_string = input(sprintf('\nEnter name for data file: '), 's')
temp_str = datestr(now,O) ; 
file_string = strcat('sim-',temp_str(1:2),temp_str(4:6),temp_str(10:11),'-',...
temp_str(13:14), temp_str(16:17)) ;
fprintf('\nData file saved as\n %s\\%s.mat', cd, file_string);
% EXPERIMENTAL/ASSIGNED VALUES: initial conditions, input vector
% --------------------------------------------------------------
% loading model inputs, generated in SIMSETUP.M
load input-vector ; % data from FININPUTS.M on mission files
load timestep
load initialstate ; % data from INITIALCONDITIONS.M on above
pitch_max = 90 ;
% RUN MODEL
%---------------------------------------------------------------------
% Initialize number of steps and storage matrix
n_steps = size(ui,2)-1;
output_table = zeros(n_steps,size(x,i)+size(ui,i)+7);
fprintf('\n Simulator running...\n');
% MAIN PROGRAM
for i = 1:n_steps,
% Print current step for error checking
if show_step == 1
if ~rem(i*10,n_steps)
fprintf(' Steps Completed : %02d %% ', i/n_steps*100);
end
end
% Store current states x(n), inputs ui(n), and time in seconds
output_table(i,1:14) = [x' ui(:,i)'] ;
output_table(i,21) = (i-1)*time_step ;
% Calculate forces, accelerations
% ** CALLS REMUS.M
% xdot(i) = f(x(i),u(i))
[dummy,forces] = remus(x,ui(:,i)'); 
% Store forces at step n
output_table(i,15:20) = [forces'];

%  RUNGE-KUTTA APPROXIMATION to calculate new states
%  NOTE: ideally, should be approximating ui values for k2,k3
%  ie (ui(:,i)+ui(:,i+1))/2
x = rk4(remus, time_step, x, ui, i);
end
% SAVE SIMULATOR OUTPUT
% model coefficients and vehicle parameters loaded in REMUS.M
%%%%% load vdata ; load vehicle_type ; load inv_mass_matrix ; load vehiclecoeffs

save(file_string, 'output_table', 'file_string', 'date_string', 'time_string', ...
'timestep', 'x', 'ui', ...
'W', 'Minv', 'B', 'm', 'g', 'rho', 'xg', 'yg', 'zg', 'xb', 'yb', 'zb', ...
'Ixx', 'Iyy', 'Izz', 'delta_max',...
'Xuu', 'Xudot', 'Xwq', 'Xqq', 'Xvr', 'Xrr', 'Xprop',  ...
'Yvv', 'Yrr', 'Yuv', 'Yvdot', 'Yrdot', 'Yur', 'Ywp', 'Ypq', 'Yuudr', ...
'Zww', 'Zqq', 'Zuw', 'Zwdot', 'Zqdot', 'Zuq', 'Zvp', 'Zrp', 'Zuuds', ...
'Kpdot', 'Kprop', 'Kpp', ...
'Mww', 'Mqq', 'Muw', 'Mwdot', 'Mqdot', 'Muq', 'Mvp', 'Mrp', 'Muuds', ...
'Nvv', 'Nrr', 'Nuv', 'Nvdot', 'Nrdot', 'Nur', 'Nwp', 'Npq', 'Nuudr');
% return to working directory
cd model ;
% save text file of ismulator inputs
if runsavedata
savedata;
end
% Plot output
figstart = input('\n Starting Figure Number ');
figstart = figstart - 1;
remusplots; simplot ; 
if run_plots 
    fmplot;
end
fprintf('\n');
return;