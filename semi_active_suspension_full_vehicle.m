function [Zdot, O_simulator, O_model] = semi_active_suspension_full_vehicle(t, z, input)
%active_suspension_quarter_car Simulator function that runs vehicle simulations
%   This is a wrapping function that is called by the numerical integrator.
%   It knows the current time-step and using it, it interpolates all the
%   inputs to be tracked. It also calculates the control action and interpolates
%   road inputs and it passes these as scalar values to the vehicle model.

%% Initialization : Augmented state variables

% Model States
% q = [z(1), z(2), z(3), z(4)]';
q = z(1:28);

ndof = input.ndof;

% Controller (Active Damper) States
temp = 2*ndof;
Z_mr1 = [z(temp + 1 : temp + size(input.cA_mr,1))];

temp = 2*ndof + length(Z_mr1);
Z_mr2 = [z(temp + 1 : temp + size(input.cA_mr,1))];

temp = 2*ndof + length(Z_mr1) + length(Z_mr2);
Z_mr3 = [z(temp + 1 : temp + size(input.cA_mr,1))];

temp = 2*ndof + length(Z_mr1) + length(Z_mr2) + length(Z_mr3);
Z_mr4 = [z(temp + 1 : temp + size(input.cA_mr,1))];

%% Initializing : Calling the vehicle model once to receive measurement outputs

[~,~,~,O_model] = vehicle_model_fw_simplified(q,input,0,0,0,0,0,0,0,0,0,0);

% Sprung mass corner displacements
zs_1 = O_model(3);
zs_2 = O_model(4);
zs_3 = O_model(5);
zs_4 = O_model(6);


% Damper piston velocities (Used for damping force saturation)
zs_dot_1 = O_model(7);
zs_dot_2 = O_model(8);
zs_dot_3 = O_model(9);
zs_dot_4 = O_model(10);


% Suspension corner deflections
suspension_deflection_1 = O_model(11);
suspension_deflection_2 = O_model(12);
suspension_deflection_3 = O_model(13);
suspension_deflection_4 = O_model(14);

% Damper corner velocities
damper_piston_velocity_1 = O_model(15);
damper_piston_velocity_2 = O_model(16);
damper_piston_velocity_3 = O_model(17);
damper_piston_velocity_4 = O_model(18);
%% Initialization : Steering Input

delta_c = interp1(input.time, input.delta, t, "pchip");

m_d_c = 0;


%% Initialization : Road Input

% z_r = interp1(input.time, input.z_r, t, 'pchip');
z_r_1 = interp1(input.time,input.z_r_1,t,"pchip");
z_r_2 = interp1(input.time,input.z_r_2,t,"pchip");
z_r_3 = interp1(input.time,input.z_r_3,t,"pchip");
z_r_4 = interp1(input.time,input.z_r_4,t,"pchip");

%% Initialization : Controll Matrices

% Sprung Mass Position Controller
% -- A -- Controller Cannonical State Matrix 
Ac_ds = input.cA_mr;

% -- B -- Controller Cannonical Input Matrix
Bc_ds = input.cB_mr;

% -- C -- Controller Cannonical Output Matrix
Cc_ds = input.cC_mr;

% -- D -- Controller Cannonical Outpu<> Input Matrix
Dc_ds = input.cD_mr;

%% Reference Signal to be tracked

% Here we will create the reference signal that the controller must track
% In our case, this implies the position of the sprung mass that must be
% tracked
% Since our goal is to keep the sprung mass as level as possible, the
% displacement of the sprung mass must be 0
% The displacement being 0 implies that the position of the sprung mass
% must not change which implies the position is equal to the steady-state
% value calculated
% zs_1_ref = z_r_1;
% zs_2_ref = z_r_2;
% zs_3_ref = z_r_3;
% zs_4_ref = z_r_4;

suspension_deflection_1_ref = 0;
suspension_deflection_2_ref = 0;
suspension_deflection_3_ref = 0;
suspension_deflection_4_ref = 0;

% z_dot_s_ref = 0;
% z_sus_def_ref = (input.zu_steady_state - input.zs_steady_state);

%% Error 

% e_zs = zs_1_ref - (z_s - input.zs_steady_state);
% e_z_dot_s = z_dot_s_ref - z_dot_s;
% e_sus_def =  z_sus_def_ref - (z_u - z_s);

e_sus_def_1 = suspension_deflection_1_ref - suspension_deflection_1;
e_sus_def_2 = suspension_deflection_2_ref - suspension_deflection_2;
e_sus_def_3 = suspension_deflection_3_ref - suspension_deflection_3;
e_sus_def_4 = suspension_deflection_4_ref - suspension_deflection_4;

error_1 = e_sus_def_1;
error_2 = e_sus_def_2;
error_3 = e_sus_def_3;
error_4 = e_sus_def_4;

%% Controller Action - Desired Controller Force

% Controller output
y_mr_1 = Cc_ds*Z_mr1 + Dc_ds*error_1;
y_mr_2 = Cc_ds*Z_mr2 + Dc_ds*error_2;
y_mr_3 = Cc_ds*Z_mr3 + Dc_ds*error_3;
y_mr_4 = Cc_ds*Z_mr4 + Dc_ds*error_4;

% Desired controller force
F_active_damper_1 = input.controller_switch * y_mr_1;
F_active_damper_2 = input.controller_switch * y_mr_2;
F_active_damper_3 = input.controller_switch * y_mr_3;
F_active_damper_4 = input.controller_switch * y_mr_4;

% kp = 400000;
% ki = 1000;
% F_active_damper = kp*e_zs ;

%% Inverse Controller Model - Realizeable Controller Force

% Minimum suppliable current (Reference - Tenecco Active Damper)
I_min = input.I_min; % A

% Maximum suppliable current (Reference - Tenecco Active Damper)
I_max = input.I_max; % A


I_required_1 = calculate_current_from_force(F_active_damper_1, damper_piston_velocity_1);
I_required_2 = calculate_current_from_force(F_active_damper_2, damper_piston_velocity_2);
I_required_3 = calculate_current_from_force(F_active_damper_3, damper_piston_velocity_3);
I_required_4 = calculate_current_from_force(F_active_damper_4, damper_piston_velocity_4);

I_possible_1 = calculate_possible_current(I_required_1,I_min,I_max);
I_possible_2 = calculate_possible_current(I_required_2,I_min,I_max);
I_possible_3 = calculate_possible_current(I_required_3,I_min,I_max);
I_possible_4 = calculate_possible_current(I_required_4,I_min,I_max);


F_active_damper_1 = input.controller_switch*calculate_force_from_current(I_possible_1, damper_piston_velocity_1);
F_active_damper_2 = input.controller_switch*calculate_force_from_current(I_possible_2, damper_piston_velocity_2);
F_active_damper_3 = input.controller_switch*calculate_force_from_current(I_possible_3, damper_piston_velocity_3);
F_active_damper_4 = input.controller_switch*calculate_force_from_current(I_possible_4, damper_piston_velocity_4);

%% Quarter car system dynamics

[Qdot, ~, ~, O_model] = vehicle_model_fw_simplified(q, input, delta_c, m_d_c, ...
                                                    F_active_damper_1, ...
                                                    F_active_damper_2, ...
                                                    F_active_damper_3, ...
                                                    F_active_damper_4, ...
                                                    z_r_1, ...
                                                    z_r_2, ...
                                                    z_r_3, ...
                                                    z_r_4);

%% Controller Dynamics

Z_mr_dot_1 = Ac_ds*Z_mr1 + Bc_ds*error_1;
Z_mr_dot_2 = Ac_ds*Z_mr2 + Bc_ds*error_2;
Z_mr_dot_3 = Ac_ds*Z_mr3 + Bc_ds*error_3;
Z_mr_dot_4 = Ac_ds*Z_mr4 + Bc_ds*error_4;

%% Augmented system dynamics

Zdot = [Qdot;
        Z_mr_dot_1;
        Z_mr_dot_2;
        Z_mr_dot_3;
        Z_mr_dot_4
        ];


%% Outputs

O_simulator = [error_1;
               error_2;
               error_3;
               error_4;
               F_active_damper_1;
               F_active_damper_2;
               F_active_damper_3;
               F_active_damper_4;
               F_active_damper_1/damper_piston_velocity_1;
               F_active_damper_2/damper_piston_velocity_2;
               F_active_damper_3/damper_piston_velocity_3;
               F_active_damper_4/damper_piston_velocity_4;
               suspension_deflection_1;
               suspension_deflection_2;
               suspension_deflection_3;
               suspension_deflection_4;
               damper_piston_velocity_1;
               damper_piston_velocity_2;
               damper_piston_velocity_3;
               damper_piston_velocity_4
               O_model(1)]';



























































































end