function [Qdot, f_qd_q_u, M, O] = quarter_car_model_linear(q, input, F_active_damper, z_r)
%quarter_car_model_linear Simplified, quarter car model
%   This is a quarter car model formulated in a linear, abstract
%   way. 
% INPUTS
%   q           State-vector in 1st order formulation which contains
%               generalized positions & velocities
%   input       Input structu which contains vehicle parameter information
%               such as mass, inertia, tire model etc. 
%   F_a         Actuator force (controller)
%   z_r         Road Inputs

% NOTE: Check the "input_script.m" to understand the inputs inside the
%       input structure

% OUTPUTS
% NOTE- The vectors and matrices that this function outputs are done so
% keeping in mind future operations that this function will be used for 
% EXAMPLE - This function may need to output controller states or estimator/observer states. The O_simulator will do that
% EXAMPLE - The vehicle model may need to output complex variables like slip angle, slip ratio etc. The O_model will do that 

%   Qdot        State vector time derivative
%   f_qd_q_u    Forces & moments vector
%   M           Mass matrix
%   O           Model Outputs


%% Initialization : State Variables

z_s = q(1);
z_u = q(2);

zs_dot = q(3);
zu_dot = q(4);
%% Initialization : Component 

% This is now a control input
d_s = input.d_s;       % Ns/m
% d_s = 3000;
m_s = input.m_s;       % kg
m_a = input.m_a;       % kg
k_s = input.k_s;       % N/m
k_t = input.k_t;       % N/m
d_t = input.d_t;       % Ns/m


%% System Dynamics

M = [m_s 0;
     0 m_a];

f_qd_q_u = [k_s*(z_u - z_s) + d_s*(zu_dot - zs_dot) + F_active_damper - input.gravity_switch*m_s*9.81;
           -k_s*(z_u - z_s) - d_s*(zu_dot - zs_dot) - F_active_damper - input.gravity_switch*m_a*9.81 + k_t*(z_r - z_u) + d_t*(- zu_dot)];

Qdot = [zs_dot;
        zu_dot;
        M\f_qd_q_u
        ];




%% Outputs

O = [F_active_damper/(zu_dot - zs_dot)];




































































end