function [I_required] = calculate_current_from_force(F_desired, damper_piston_velocity)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

v = damper_piston_velocity*1;

% c0 = 473.89;
% c1 = 560.46;
% c2 = -76.17;
% c3 = -26.41;
% c4 = 4.01;
% c5 = 0.89;
% c6 = -0.09;
% c7 = -0.02;
% c8 = 9.98e-4;
% c9 = 1.49e-4;
% c10 = -3.96e-6;
% c11 = -5.32e-7;
% 
% b0 = -91.8;
% b1 = 121.49;
% b2 = -6.95;
% b3 = -7.06;
% b4 = 0.45;
% b5 = 0.26;
% b6 = -0.01;
% b7 = -5.1e-3;
% b8 = 1.34e-4;
% b9 = 4.71e-5;
% b10 = -5.51e-7;
% b11 = -1.68e-7;

c0 = -659.4;
c1 = 8.995;
c2 = 0.1062;
c3 = -1.584e-4;
c4 = -5.908e-6;
c5 = 1.137e-9;
c6 = 1.087e-10;
c7 = 0;
c8 = 0;
c9 = 0;
c10 = 0;
c11 = 0;

b0 = -371.8;
b1 = 6.205;
b2 = 0.03728;
b3 = -3.487e-4;
b4 = -2.767e-6;
b5 = 6.924e-9;
b6 = 5.604e-11;
b7 = 0;
b8 = 0;
b9 = 0;
b10 = 0;
b11 = 0;

F_base = (0*b0*v^0 + b1*v^1 + b2*v^2 + b3*v^3 + b4*v^4 + b5*v^5 + b6*v^6 + b7*v^7 + b8*v^8 + b9*v^9 + b10*v^10 + b11*v^11);

I_required = (F_desired - F_base)/(0*c0*v^0 + c1*v^1 + c2*v^2 + c3*v^3 + c4*v^4 + c5*v^5 + c6*v^6 + c7*v^7 + c8*v^8 + c9*v^9 + c10*v^10 + c11*v^11);

I_required = (I_required);

end