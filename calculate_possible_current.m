function [I_possible] = calculate_possible_current(I_required,I_min, I_max)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

I_possible = I_required;

%% Check 1
if isnan(I_required) || isinf(I_required)
    I_possible = 0;
end


%% Check 2

if abs(I_required) <= I_min 
    I_possible = I_min;
elseif abs(I_required) >= I_max
    I_possible = I_max;
end



end