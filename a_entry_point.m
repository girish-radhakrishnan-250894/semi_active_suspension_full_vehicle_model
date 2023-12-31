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
% clc;
clear;
addpath(genpath(pwd));

%% Inputs

input_script;

input_script_controller;

%% Defining the system and controller properties


% Creating the augmentat state-vector
% The angular velocities of the wheel required a logical initial guess
v_guess = input.u_start;
omega_y_1_guess = v_guess/input.r_01; % Simplified estimation of wheel angular velocity using vehicle speed and unloaded radius
omega_y_2_guess = v_guess/input.r_02;
omega_y_3_guess = v_guess/input.r_03;
omega_y_4_guess = v_guess/input.r_04;

% State vector initial conditions
q0 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 input.u_start 0 0 0 0 0 0 0 0 0 omega_y_1_guess omega_y_2_guess omega_y_3_guess omega_y_4_guess];
Z0 = [q0'; 
      zeros(1, size(cA_mr, 1))'; 
      zeros(1, size(cA_mr, 1))'; 
      zeros(1, size(cA_mr, 1))'; 
      zeros(1, size(cA_mr, 1))']';
%% Running the Simulation
% Integration options
opts = odeset("RelTol",1e-3,'MaxStep',0.01);

[t,Z] = ode45(@(t,Z)semi_active_suspension_full_vehicle(t,Z,input), [0 input.time(end)], Z0);

% Output matrix initialization
n_outputs = 22;
O = zeros(length(t),n_outputs);
for i=1:length(Z)
    [~, O(i,:)] = semi_active_suspension_full_vehicle(t(i),Z(i,:)',input);
end

%% Plotting the Simulation Results
col = 'r';
% Road Input
figure
subplot(2,2,1)
plot(input.x_r_1,input.z_r_1*1000 , col)
legend('z_u_1')

subplot(2,2,2)
plot(input.x_r_2,input.z_r_2*1000 , col)
legend('z_u_2')

subplot(2,2,3)
plot(input.x_r_3,input.z_r_3*1000 , col)
legend('z_u_3')

subplot(2,2,4)
plot(input.x_r_4,input.z_r_4*1000 , col)
legend('z_u_4')


% Sprung Mass Acceleration
% figure
% plot(t,O(:,22) , col)
% legend('$\ddot{z}_{s}$','Interpreter','latex')
% grid minor
% set(findall(gcf,'-property','FontSize'),'FontSize',16)
figure
z_ddot = [0;diff(diff(Z(:,3)))]./diff(t);
plot(t,[0;z_ddot], col)
legend('$\ddot{z}_{s}$','Interpreter','latex')
grid minor
set(findall(gcf,'-property','FontSize'),'FontSize',16)


% Sprung Mass Displacement
figure
plot(t,Z(:,3)*1000 , col)
legend('z_s')
grid minor
set(findall(gcf,'-property','FontSize'),'FontSize',16)

% Sprung Mass Roll
figure
plot(t,rad2deg(Z(:,4)) , col)
legend('theta')
grid minor
set(findall(gcf,'-property','FontSize'),'FontSize',16)

% Sprung Mass Pitch
figure
plot(t,rad2deg(Z(:,5)) , col)
legend('phi')
grid minor
set(findall(gcf,'-property','FontSize'),'FontSize',16)


% Unsprung Mass Displacement
figure
subplot(2,2,1)
plot(t,Z(:,7)*1000 , col)
legend('z_u_1')

subplot(2,2,2)
plot(t,Z(:,8)*1000 , col)
legend('z_u_2')

subplot(2,2,3)
plot(t,Z(:,9)*1000 , col)
legend('z_u_3')

subplot(2,2,4)
plot(t,Z(:,10)*1000 , col)
legend('z_u_4')









% Suspension Deflections
figure
subplot(2,2,1)
plot(t,O(:,13)*1000 , col)
legend('\Delta_s_1')

subplot(2,2,2)
plot(t,O(:,14)*1000 , col)
legend('\Delta_s_2')

subplot(2,2,3)
plot(t,O(:,15)*1000 , col)
legend('\Delta_s_3')

subplot(2,2,4)
plot(t,O(:,16)*1000 , col)
legend('\Delta_s_4')






% Damper Shaft Velocities
figure
subplot(2,2,1)
plot(t,O(:,17)*1000 , col)
legend('$\dot{\Delta}_{s_1}$','Interpreter','latex')

subplot(2,2,2)
plot(t,O(:,18)*1000 , col)
legend('$\dot{\Delta}_{s_2}$','Interpreter','latex')

subplot(2,2,3)
plot(t,O(:,19)*1000 , col)
legend('$\dot{\Delta}_{s_3}$','Interpreter','latex')

subplot(2,2,4)
plot(t,O(:,20)*1000 , col)
legend('$\dot{\Delta}_{s_4}$','Interpreter','latex')








% Error Signals
figure
subplot(2,2,1)
plot(t,O(:,1) , col)
legend('e_{zs_1}')

subplot(2,2,2)
plot(t,O(:,2) , col)
legend('e_{zs_2}')

subplot(2,2,3)
plot(t,O(:,3) , col)
legend('e_{zs_3}')

subplot(2,2,4)
plot(t,O(:,4) , col)
legend('e_{zs_4}')







% Control Action
figure
subplot(2,2,1)
plot(t,O(:,5)*1000 , col)
legend('F_{mr_1}')

subplot(2,2,2)
plot(t,O(:,6)*1000 , col)
legend('F_{mr_2}')

subplot(2,2,3)
plot(t,O(:,7)*1000 , col)
legend('F_{mr_3}')

subplot(2,2,4)
plot(t,O(:,8)*1000 , col)
legend('F_{mr_4}')





% MR Damping Coefficient
figure
subplot(2,2,1)
plot(t,O(:,9)*1000 , col)
legend('ds_{mr_1}')

subplot(2,2,2)
plot(t,O(:,10)*1000 , col)
legend('ds_{mr_2}')

subplot(2,2,3)
plot(t,O(:,11)*1000 , col)
legend('ds_{mr_3}')

subplot(2,2,4)
plot(t,O(:,12)*1000 , col)
legend('ds_{mr_4}')

































