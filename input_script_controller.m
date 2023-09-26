%% MR Damper Current Limitations 
input.I_min = 0;    % A
input.I_max = 3;    % A

%% Load Controller for each Corner

% Same controller used for each corner 
load('active_damper_suspension_deflection_control_it1.mat')

% Convert controller transfer function to state-space
[cA_mr,cB_mr,cC_mr,cD_mr] = tf2ss(cell2mat(shapeit_data.C_tf.Numerator),cell2mat(shapeit_data.C_tf.Denominator));

input.cA_mr = cA_mr;
input.cB_mr = cB_mr;
input.cC_mr = cC_mr;
input.cD_mr = cD_mr;

input.controller_switch = 1;
input.gravity_switch = 0;

input.n_states_controloler = size(cA_mr,1);

if (input.controller_switch == 1)
    input.d_s = 0;
end