%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%% TRIGGER SCRIPT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% -> This is the main script of this model and of this repository.

% -> This script is the first script that must be run.

% -> This script shall trigger all the remaining necessary scripts. 

% -> To edit the simulation inputs such as the steering angle, please 
%    navigate to the script called F02_INPUT_SCRIPT. 

% -> IMPORTANT :- The order in which the scripts are called below are very
%                 important. Changing the order will result in faulty
%                 simulation
clc;
clear;
addpath(genpath(pwd));

%% Inputs

input_script;

%% Defining the system and controller properties
% Integration options
opts = odeset("RelTol",1e-6,'MaxStep',0.005);

% Creating the augmentat state-vector
Z0 = [input.zs_steady_state input.zu_steady_state 0 0 zeros(1, size(cA_ds, 1))];

%% Creating the Simulation Inputs
u = 5;                  % x velocity in m s-1
simulation_time = 10;   % simulation time in seconds

% mode 1 - flat road
% mode 2 - step input of 10 mm
% mode 3 - random road input
% mode 4 - speedbump
road_mode = 4;

% [input.time, input.z_r] = road_input_selector(simulation_time, road_mode, u);
% Smooth Pulse Road Input
input.time =   [0 2 4 6 8 10 12 14 16 18 20]/4;
input.z_r  = 20*[0 0 5 5 5  5  5  5  5  5  5]*1e-3; % Your equations on the speed


% Theoretical Step Road Input


% Synthetic Random Road Input

%% Running the Simulation

[t,Z] = ode15s(@(t,Z)semi_active_suspension_full_vehicle(t,Z,input), [0 input.time(end)], Z0, opts);

% Output matrix initialization
n_outputs = 4;
O = zeros(length(t),n_outputs);
for i=1:length(Z)
    [~, O(i,:)] = semi_active_suspension_full_vehicle(t(i),Z(i,:)',input);
end

%% Plotting the Simulation Results

% Road Input
figure
plot (input.time, input.z_r*1000, 'Color', [0.75, 0.0, 0.0],'LineWidth',2.0) 
title('Road Input', 'FontSize',20)
xlabel ('Time (s)')
ylabel ('Road Input z_{r} [mm]')
grid on


% Sprung Mass Displacement
figure
plot(t,Z(:,1)*1000,'k')
legend('z_s')
grid minor
set(findall(gcf,'-property','FontSize'),'FontSize',16)

% Unsprung Mass Displacement
figure
plot(t,Z(:,2)*1000,'k')
legend('z_u')
grid minor
set(findall(gcf,'-property','FontSize'),'FontSize',16)

% Sprung mass displacement error
figure
plot(t,O(:,1)*1,'k')
legend('e_zs')
grid minor
set(findall(gcf,'-property','FontSize'),'FontSize',16)


% Controller Force Input
figure
plot(t,O(:,3),'k')
legend('F_controller')
grid minor
set(findall(gcf,'-property','FontSize'),'FontSize',16)

% Damping Coefficient of MR Damper
figure
plot(t,O(:,4))
legend("MR Damping Coefficient")














































