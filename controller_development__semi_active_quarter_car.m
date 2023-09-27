clear all
close all


%% Parameter Initialization
m_s = 510; 
m_a = 60; 
k_s = 52500; 
k_t = 200000;
d_s = 1240; 
d_t = 100;
g   = 9.81; 

%% road irregularities (a flat road for the moment)
t   =[0 5 10];  %tme vector 
z_r = [0 5 10];

%% initializing the state-space model - with active element
A = [0         0                 1           0;
     0         0                 0           1;
    -k_s/m_s   k_s/m_s          -d_s/m_s     d_s/m_s;
     k_s/m_a -(k_s + k_t)/m_a    d_s/m_a   -(d_s + d_t)/m_a];


B = [0; 0 ; 1/m_s; -1/m_a];


C = [-1 1 0 0];

D= [0]';

%state-space model
sys=ss(A,B,C,D); 
[num,den] = ss2tf(A,B,C,D);

z_ds_qcar = tf(num,den)

