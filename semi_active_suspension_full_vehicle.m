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

% Controller (Active Damper) States
temp = 2*ndof;
Z_zs1 = [Q(temp + 1 : temp + size(input.cA_F1,1))];

temp = 2*ndof + length(Z_zs1);
Z_zs2 = [Q(temp + 1 : temp + size(input.cA_F2,1))];

temp = 2*ndof + length(Z_zs1) + length(Z_zs2);
Z_zs3 = [Q(temp + 1 : temp + size(input.cA_F3,1))];

temp = 2*ndof + length(Z_zs1) + length(Z_zs2) + length(Z_zs3);
Z_zs4 = [Q(temp + 1 : temp + size(input.cA_F4,1))];


% Displacements
z_s = z(1);
z_u = z(2);

% Velocities
z_dot_s = z(3);
z_dot_u = z(4);

% Damper piston velocity (Used for damping force saturation)
damper_piston_velocity = z_dot_u - z_dot_s;

%% Initialization : Road Input

z_r = interp1(input.time, input.z_r, t, 'pchip');

%% Initialization : Controll Matrices

% Sprung Mass Position Controller
% -- A -- Controller Cannonical State Matrix 
Ac_ds = input.cA_ds;

% -- B -- Controller Cannonical Input Matrix
Bc_ds = input.cB_ds;

% -- C -- Controller Cannonical Output Matrix
Cc_ds = input.cC_ds;

% -- D -- Controller Cannonical Outpu<> Input Matrix
Dc_ds = input.cD_ds;

%% Reference Signal to be tracked

% Here we will create the reference signal that the controller must track
% In our case, this implies the position of the sprung mass that must be
% tracked
% Since our goal is to keep the sprung mass as level as possible, the
% displacement of the sprung mass must be 0
% The displacement being 0 implies that the position of the sprung mass
% must not change which implies the position is equal to the steady-state
% value calculated
z_s_ref = z_r;
z_dot_s_ref = 0;
z_sus_def_ref = (input.zu_steady_state - input.zs_steady_state);

%% Error 

e_zs = z_s_ref - (z_s - input.zs_steady_state);
e_z_dot_s = z_dot_s_ref - z_dot_s;
e_sus_def =  z_sus_def_ref - (z_u - z_s);

error = e_zs;

%% Controller Action - Desired Controller Force

% Controller output
y_cont = Cc_ds*Z_cont_corner_1 + Dc_ds*error;

% Desired controller force
F_active_damper = input.controller_switch * y_cont;

% kp = 400000;
% ki = 1000;
% F_active_damper = kp*e_zs ;

%% Inverse Controller Model - Realizeable Controller Force

% Minimum suppliable current (Reference - Tenecco Active Damper)
I_min = input.I_min; % A

% Maximum suppliable current (Reference - Tenecco Active Damper)
I_max = input.I_max; % A


I_required = calculate_current_from_force(F_active_damper, damper_piston_velocity);

if isnan(I_required) || isinf(I_required)
    I_required = 0;
end

if abs(I_required) < I_min 
    I_required = I_min;
elseif abs(I_required) > I_max
    I_required = I_max;
end

F_active_damper = input.controller_switch*calculate_force_from_current(I_required, damper_piston_velocity);

%% Quarter car system dynamics

[Qdot, ~, ~, O_model] = quarter_car_model_linear(q, input, F_active_damper, z_r);

%% Controller Dynamics

Z_dot_cont_qcar = Ac_ds*Z_cont_corner_1 + Bc_ds*error;

%% Augmented system dynamics

Zdot = [Qdot;
        Z_dot_cont_qcar
        ];


%% Outputs

O_simulator = [error;
               Qdot(3);
               F_active_damper;
               O_model(1)]';



























































































end