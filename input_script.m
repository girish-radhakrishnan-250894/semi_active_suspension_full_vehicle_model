%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% INPUT SCRIPT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% ALL VALUES IN SI UNITS [Kilogram, Meter, Second]



% -> This script initializes all the inputs required by the four-wheel
%    model
% -> This includes all the mass/dimensionl inputs, component inputs and
%    simulation inputs. 
addpath(genpath(pwd));

%% Simulation Type selection

% -> Steering Types:
%    *0 - straight
%    *1 - ramp
%    *2 - J turn
%    *3 - double lane change
%    *4 - track replay
%    *5 - spiral
%    *6 - chirp
% -> Acceleration Types:
%    *0 - Zero Speed (movingVehicle = false)
%    *1 - constant speed
%    *2 - acceleration
%    *3 - decceleration puse
%    *4 - track replay
%    *5 - acceleration + decceleration
%    *6 - deceleration ramp

% Controller Gain Scaling
input.P_v_scaler = 0.003; % P_v is auto-set as 4.04007*m_s*g
%% INPUT :- STEERING

input.delta = 0*[0 0 0 0 deg2rad(1) deg2rad(1) 0 0 -deg2rad(1) -deg2rad(1) 0 0];
input.time  =   [0 1 2 3 4 5 6 7 8 9 10 11];
%% INPUT :- SPEED

input.u_start = 50/3.6;
input.u_max = 150/3.6;
%% INPUT :- Torque Distribution

input.Md_1_dis_acc = 0.0 + 0.0; % [-]
input.Md_2_dis_acc = 0.0 + 0.0; % [-]
input.Md_3_dis_acc = 0.5 + 0.0; % [-]
input.Md_4_dis_acc = 0.5 + 0.0; % [-]

input.Md_1_dis_brake = 0.325 + 0.0; % [-]
input.Md_2_dis_brake = 0.325 + 0.0; % [-]
input.Md_3_dis_brake = 0.175 + 0.0; % [-]
input.Md_4_dis_brake = 0.175 + 0.0; % [-]
%% INPUT :- PROPERTIES : MASS & INERTIA

sc = 1;
input.m_s = sc*1110;             % Sprung Mass [kg]
input.J_x = sc*627;              % Sprung Mass Inertia : X-Axes (Roll Inertia) [kgm^2]
input.J_y = sc*2302;             % Sprung Mass Inertia : Y-Axes (Pitch Inertia) [kgm^2]
input.J_z = sc*2394;             % Sprung Mass Inertia : Z-Axes (Yaw Inertia) [kgm^2]
input.m_u_1 = sc*52.5;             % Unsprung Mass 1 [kg] 
input.m_u_2 = sc*52.5;             % Unsprung Mass 2 [kg]
input.m_u_3 = sc*37.5;             % Unsprung Mass 3 [kg]
input.m_u_4 = sc*37.5;             % Unsprung Mass 4 [kg]
input.I_yp_1 = 3.705;               % Unsprung Inertia 1 : Y-Axis (Rotational Ineretia) [kgm^2]
input.I_yp_2 = 3.705;         % Unsprung Inertia 2 : Y-Axis (Rotational Ineretia) [kgm^2]
input.I_yp_3 = 4.494;         % Unsprung Inertia 3 : Y-Axis (Rotational Ineretia) [kgm^2]
input.I_yp_4 = 4.494;         % Unsprung Inertia $ : Y-Axis (Rotational Ineretia) [kgm^2]
input.h_cg__0  = 0.44181;       % Sprung Mass CG Height [m]


%% INPUT :- PROPERTIES : DIMENSIONS

% Dimensions :- CG to corner 'i'
%            a indicates longitudinal distance
%            s indicates lateral distance
%            a1__c indicates X coordinate of corner 1 (front left) from CG
%                  in chassis frame of reference

input.weight_distribution = 0.6; 

% Longitudinal dimensions
kinematics.wheelbase = 2.672;
kinematics.a = (1-input.weight_distribution)*kinematics.wheelbase;
kinematics.b = input.weight_distribution*kinematics.wheelbase;

input.a_1 = + kinematics.a;
input.a_2 = + kinematics.a;
input.a_3 = - kinematics.b;
input.a_4 = - kinematics.b;


% Lateral dimensions
input.s_1 = + 0.795;
input.s_2 = - 0.795;
input.s_3 = + 0.805;
input.s_4 = - 0.805;

% Component Dimension :- Wheel Unloaded Radii - Corner 'i'
input.r_01 = 0.3277;
input.r_02 = 0.3277; 
input.r_03 = 0.3277; 
input.r_04 = 0.3277; 

% Component Dimension :- Spring Free Length
input.l_01 = 0.35;
input.l_02 = 0.35;
input.l_03 = 0.35;
input.l_04 = 0.35;

%% INPUT :- PROPERTIES : COMPONENT

% Spring & Damper
input.k_s = 60000;
input.d_s = 9000;

% Tire (Vertical)
input.k_t = 250000;

% Tire (Lateral & Longitudinal)
input.tirFile_1 = mfeval.readTIR('MagicFormula61_Parameters.tir');
input.tirFile_2 = mfeval.readTIR('MagicFormula61_Parameters.tir');
input.tirFile_3 = mfeval.readTIR('MagicFormula61_Parameters.tir');
input.tirFile_4 = mfeval.readTIR('MagicFormula61_Parameters.tir');

%% INPUT :- PROPERTIES : MISC.
input.i_s = 17.842;               % Steering Ratio [-]

