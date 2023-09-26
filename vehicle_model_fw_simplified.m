function [Qdot,f_qd_q_u,M,O] = vehicle_model_fw_simplified(q,input,delta_c, m_d_c, F_a_1, F_a_2, F_a_3, F_a_4, z_r_1, z_r_2, z_r_3, z_r_4)
%vehicle_model_fw_simplified Simplified, abstract, four-wheel vehicle model
%   This is a four-wheel vehicle model formulated in a simplified, abstract
%   way. Small angle approximations are used substantially in this
%   formulation. Slip angle/ratio are calculated using the slip of the
%   chassis corner instead of slip of the contact patch. 
% INPUTS
%   q           State-vector in 1st order formulation which contains
%               generalized positions & velocities
%   input       Input structu which contains vehicle parameter information
%               such as mass, inertia, tire model etc. 
%   delta_c     Steering input command in radian
%   m_d_c       Accelerating/Braking torque input command in radian

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


% NOTE
% __c   indicates chassis frame of refernece
% __0   indicates world frame of reference
% __i   indicates i frame of reference (i = 1,2,3,4)
%% Initialization : System States
x = q(1);
y = q(2);
z = q(3);
theta = q(4);
phi = q(5);
psi = q(6);
z_1 = q(7);
z_2 = q(8);
z_3 = q(9);
z_4 = q(10);
int_omega_1 = q(11);
int_omega_2 = q(12);
int_omega_3 = q(13);
int_omega_4 = q(14);

x_d = q(15);
y_d = q(16);
z_d = q(17);
co_omega__0_1 = q(18);
co_omega__0_2 = q(19);
co_omega__0_3 = q(20);
z_d_1 = q(21);
z_d_2 = q(22);
z_d_3 = q(23);
z_d_4 = q(24);
omega_1 = q(25);
omega_2 = q(26);
omega_3 = q(27);
omega_4 = q(28);

%% Initialization : Vehicle Input Structure

in = input;

%% Rotation Matrices - Chassis 

A_a0 = [cos(psi) sin(psi) 0;
       -sin(psi) cos(psi) 0;
        0            0    1];

A_ba = [cos(phi) 0 -sin(phi);
        0        1         0;
        sin(phi) 0 cos(phi)];

A_cb = [1        0            0;
        0 cos(theta) sin(theta);
        0 -sin(theta) cos(theta)];

A_c0 = A_cb*A_ba*A_a0;

%% Rotation Matrices - NSM - Corner 1
A_a10 = [cos(psi + delta_c) sin(psi + delta_c) 0;
        -sin(psi + delta_c) cos(psi + delta_c) 0;
         0                     0               1]; % Assuming no Ackermann steering 

% A_b1a1 = A_ba; % Assuming NSM will pitch at same angle as chassis

% A_1b1 = A_cb; % Ignoring camber and assuming NSM will roll at same angle as chassis

A_10 = A_cb*A_ba*A_a10;

%% Rotation Matrices - NSM - Corner 2
A_a20 = A_10; % Assuming no Ackermann steering 

% A_b2a2 = A_ba; % Assuming NSM will pitch at same angle as chassis

% A_2b2 = A_cb; % Ignoring camber and assuming NSM will roll at same angle as chassis

A_20 = A_cb*A_ba*A_a10;

%% Rotation Matrices - NSM - Corner 3
% A_a30 = A_a0; Assuming no rear wheel steering

% A_b3a3 = A_ba; % Assuming NSM will pitch at same angle as chassis

% A_3b3 = A_cb; % Ignoring camber and assuming NSM will roll at same angle as chassis

A_30 = A_c0;

%% Rotation Matrices - NSM - Corner 4
% A_a40 = A_a0; Assuming no rear wheel steering

% A_b4a4 = A_ba; % Assuming NSM will pitch at same angle as chassis

% A_4b4 = A_cb; % Ignoring camber and assuming NSM will roll at same angle as chassis

A_40 = A_c0;

%% Angular Velocity Vector

co_omega__0 = [co_omega__0_1 co_omega__0_2 co_omega__0_3]'; % World Frame of reference

co_omega__c = (co_omega__0'*A_c0')'; % Chassis frame of reference

r = co_omega__c(3);

%% Cardan angle velocities (rate of change of cardan angles)

% Since the equations of motion are calculated using the angular velocity
% vector, integration of this vector will NOT yield the cardan angles
% Integrating the calculations below is what will yield the cardan angles
TaitBryantAngle_d__0 = [(co_omega__0_1*cos(psi) + co_omega__0_2*sin(psi))/cos(phi);
                         co_omega__0_2*cos(psi) - co_omega__0_1*sin(psi);
                        (co_omega__0_3*cos(phi) + co_omega__0_1*cos(psi)*sin(phi) + co_omega__0_2*sin(phi)*sin(psi))/cos(phi)];

theta_dot = TaitBryantAngle_d__0(1);
phi_dot = TaitBryantAngle_d__0(2);
psi_dot = TaitBryantAngle_d__0(3);

%% Chassis CG Velocity
r_cm_d__0 = [x_d y_d z_d]'; % World frame

r_cm_d__c = (r_cm_d__0'*A_c0')'; % Chassis frame

u = r_cm_d__c(1);
v = r_cm_d__c(2);
w = r_cm_d__c(3);

%% Spring Deflections
zs_1 = -(z + in.a_1*phi - in.s_1*theta);
zs_2 = -(z + in.a_2*phi - in.s_2*theta);
zs_3 = -(z + in.a_3*phi - in.s_3*theta);
zs_4 = -(z + in.a_4*phi - in.s_4*theta);

Delta_s_1 = z_1 - z + in.a_1*phi - in.s_1*theta;
Delta_s_2 = z_2 - z + in.a_2*phi - in.s_2*theta;
Delta_s_3 = z_3 - z + in.a_3*phi - in.s_3*theta;
Delta_s_4 = z_4 - z + in.a_4*phi - in.s_4*theta;

% Spring Length
l_1__c = in.l_01 - Delta_s_1; % Assuming spring deflection is same in chassis and world frame (small angle)
l_2__c = in.l_02 - Delta_s_2; % Assuming spring deflection is same in chassis and world frame (small angle)
l_3__c = in.l_03 - Delta_s_3; % Assuming spring deflection is same in chassis and world frame (small angle)
l_4__c = in.l_04 - Delta_s_4; % Assuming spring deflection is same in chassis and world frame (small angle)

%% Damper Velocities
zs_dot_1 = -(z_d + in.a_1*phi_dot - in.s_1*theta_dot);
zs_dot_2 = -(z_d + in.a_2*phi_dot - in.s_2*theta_dot);
zs_dot_3 = -(z_d + in.a_3*phi_dot - in.s_3*theta_dot);
zs_dot_4 = -(z_d + in.a_4*phi_dot - in.s_4*theta_dot);

Delta_d_s_1 = z_d_1 - z_d + in.a_1*phi_dot - in.s_1*theta_dot;
Delta_d_s_2 = z_d_2 - z_d + in.a_2*phi_dot - in.s_2*theta_dot;
Delta_d_s_3 = z_d_3 - z_d + in.a_3*phi_dot - in.s_3*theta_dot;
Delta_d_s_4 = z_d_4 - z_d + in.a_4*phi_dot - in.s_4*theta_dot;

%% Tire Deflection

if z_r_2 ~=0
    wejknf=1;
end
Delta_t_1 = z_r_1 - z_1;
Delta_t_2 = z_r_2 - z_2;
Delta_t_3 = z_r_3 - z_3;
Delta_t_4 = z_r_4 - z_4;

% Tire Loaded Radius
r_L_1 = in.r_01 - Delta_t_1;
r_L_2 = in.r_02 - Delta_t_2;
r_L_3 = in.r_03 - Delta_t_3;
r_L_4 = in.r_04 - Delta_t_4;


%% Slip Angles

% Corner 1
Vx_1    =  (u - in.s_1*r)*cos(delta_c) + (v + in.a_1*r)*sin(delta_c); % Longitudinal velocity at corner 1
V_sy_1  = -(u - in.s_1*r)*sin(delta_c) + (v + in.a_1*r)*cos(delta_c); % Lateral velocity at corner 1 - lateral slip 
alpha_1__1 = -(V_sy_1/Vx_1); % slip angle
kappa_1__1 = -(Vx_1 - omega_1*r_L_1)/(Vx_1); % slip ratio. longitudinal slip = (Vx1 - omega_1*r_L_1)

% Corner 2
Vx_2    =  (u - in.s_2*r)*cos(delta_c) + (v + in.a_2*r)*sin(delta_c);
V_sy_2  = -(u - in.s_2*r)*sin(delta_c) + (v + in.a_2*r)*cos(delta_c);
alpha_2__2 = -(V_sy_2/Vx_2);
kappa_2__2 = -(Vx_2 - omega_2*r_L_2)/(Vx_2);

% Corner 3
Vx_3    =  (u - in.s_3*r)*cos(0) + (v + in.a_3*r)*sin(0);
V_sy_3  = -(u - in.s_3*r)*sin(0) + (v + in.a_3*r)*cos(0);
alpha_3__3 = -(V_sy_3/Vx_3);
kappa_3__3 = -(Vx_3 - omega_3*r_L_3)/(Vx_3);

% Corner 3
Vx_4    =  (u - in.s_4*r)*cos(0) + (v + in.a_4*r)*sin(0);
V_sy_4  = -(u - in.s_4*r)*sin(0) + (v + in.a_4*r)*cos(0);
alpha_4__4 = -(V_sy_4/Vx_4);
kappa_4__4 = -(Vx_4 - omega_4*r_L_4)/(Vx_4);

%% Vector Formulation : Chassis CG to Contact Patch 

% Corner 1
r_cp1_cm__c = [in.a_1 in.s_1 -(l_1__c + r_L_1)]'; % Assuming r_L_1 is in chassis frame (small angle)
r_cp2_cm__c = [in.a_2 in.s_2 -(l_2__c + r_L_2)]'; % Assuming r_L_2 is in chassis frame (small angle)
r_cp3_cm__c = [in.a_3 in.s_3 -(l_3__c + r_L_3)]'; % Assuming r_L_3 is in chassis frame (small angle)
r_cp4_cm__c = [in.a_4 in.s_4 -(l_4__c + r_L_4)]'; % Assuming r_L_4 is in chassis frame (small angle)

r_cp1_cm__0 = (r_cp1_cm__c'*A_c0)';
r_cp2_cm__0 = (r_cp2_cm__c'*A_c0)';
r_cp3_cm__0 = (r_cp3_cm__c'*A_c0)';
r_cp4_cm__0 = (r_cp4_cm__c'*A_c0)';

%% Forces : Spring
F_s1__0 = [0 0 in.k_s * Delta_s_1]';
F_s2__0 = [0 0 in.k_s * Delta_s_2]';
F_s3__0 = [0 0 in.k_s * Delta_s_3]';
F_s4__0 = [0 0 in.k_s * Delta_s_4]';

%% Forces : Damper
F_d1__0 = [0 0 in.d_s * Delta_d_s_1]';
F_d2__0 = [0 0 in.d_s * Delta_d_s_2]';
F_d3__0 = [0 0 in.d_s * Delta_d_s_3]';
F_d4__0 = [0 0 in.d_s * Delta_d_s_4]';

%% Forces : Tire (Vertical)
F_kt1__0 = [0 0 in.k_t * Delta_t_1]';
F_kt2__0 = [0 0 in.k_t * Delta_t_2]';
F_kt3__0 = [0 0 in.k_t * Delta_t_3]';
F_kt4__0 = [0 0 in.k_t * Delta_t_4]';

%% Forces : Tire (Lateral & Longitudinal)
% NOTE: An important point here is that the tire must be flipped before
% using on the opposite side of the axle
% NOTE: Another important point is that the slip dynamics are in ISO frame
% of reference

tireFlip = -1; % Tire flipping involves changing sign of SA and FY 
mf_input_1 = [F_kt1__0(3), kappa_1__1, -alpha_1__1,          0, 0, omega_1*r_L_1, 180000, omega_1];
mf_input_2 = [F_kt2__0(3), kappa_2__2, -tireFlip*alpha_2__2, 0, 0, omega_2*r_L_2, 180000, omega_2];
mf_input_3 = [F_kt3__0(3), kappa_3__3, -alpha_3__3,          0, 0, omega_3*r_L_3, 180000, omega_3];
mf_input_4 = [F_kt4__0(3), kappa_4__4, -tireFlip*alpha_4__4, 0, 0, omega_4*r_L_4, 180000, omega_4];

mf_output_1 = mfeval(input.tirFile_1, mf_input_1, 211);
mf_output_2 = mfeval(input.tirFile_2, mf_input_2, 211);
mf_output_3 = mfeval(input.tirFile_3, mf_input_3, 211);
mf_output_4 = mfeval(input.tirFile_4, mf_input_4, 211);

F_cp1__1 = [mf_output_1(1), mf_output_1(2) 0]';
F_cp2__2 = [mf_output_2(1), tireFlip*mf_output_2(2) 0]';
F_cp3__3 = [mf_output_3(1), mf_output_3(2) 0]';
F_cp4__4 = [mf_output_4(1), tireFlip*mf_output_4(2) 0]';

F_cp1__0 = (F_cp1__1'*A_a10)';
F_cp2__0 = (F_cp2__2'*A_a20)';
F_cp3__0 = (F_cp3__3'*A_a0)';
F_cp4__0 = (F_cp4__4'*A_a0)';

%% Matrix Formulation : Mass & Inertia Matrix

% Mass Matrix :- Suspended Mass
MsMat = [in.m_s 0      0;
         0      in.m_s 0; 
         0      0      in.m_s];

% Inertia Matrix :- Chassis Frame       
JMat__c = [in.J_x 0      0;
           0      in.J_y 0;
           0      0      in.J_z];
%               :- World Frame       
JMat__0 = (A_c0'*JMat__c*A_c0);

% Mass Matrix :- Unsprung Mass
MuMat = [in.m_u_1 0        0        0;
         0        in.m_u_2 0        0;
         0        0        in.m_u_3 0;
         0        0        0        in.m_u_4];

% Inertia Matrix : Wheel :- Wheel Frame
JMat_y__i = [in.I_yp_1 0         0         0;
             0         in.I_yp_2 0         0;
             0         0         in.I_yp_3 0;
             0         0         0         in.I_yp_4];

JMat = JMat__0;     
% Mass Matrix :- Concatenated     
M_Mat__0 = [MsMat,                zeros(size(JMat__0)),   zeros(3,4),   zeros(3,4);
            zeros(size(MsMat)),   JMat__0,                zeros(3,4),   zeros(3,4);
            zeros(4,3),           zeros(4,3),          MuMat,           zeros(4,4);
            zeros(4,3),           zeros(4,3),          zeros(4,4),      JMat_y__i;
            ];

%% Force Summation

g = -input.gravity_switch*9.81;

% Chassis
F_sm1__0 = F_s1__0 + F_d1__0 + F_cp1__0 + [0 0 F_a_1]';
F_sm2__0 = F_s2__0 + F_d2__0 + F_cp2__0 + [0 0 F_a_2]';
F_sm3__0 = F_s3__0 + F_d3__0 + F_cp3__0 + [0 0 F_a_3]';
F_sm4__0 = F_s4__0 + F_d4__0 + F_cp4__0 + [0 0 F_a_4]';
F_sm_g__0 = [0 0 in.m_s*g]';

% NSM - Corner 1
F_nsm1__0_3 = -F_s1__0(3) - F_d1__0(3) + F_kt1__0(3) - F_a_1 + in.m_u_1*g; % Only Z direction for NSM

% NSM - Corner 2
F_nsm2__0_3 = -F_s2__0(3) - F_d2__0(3) + F_kt2__0(3) - F_a_2 + in.m_u_2*g; % Only Z direction for NSM

% NSM - Corner 3
F_nsm3__0_3 = -F_s3__0(3) - F_d3__0(3) + F_kt3__0(3) - F_a_3 + in.m_u_3*g; % Only Z direction for NSM

% NSM - Corner 4
F_nsm4__0_3 = -F_s4__0(3) - F_d4__0(3) + F_kt4__0(3) - F_a_4 + in.m_u_4*g; % Only Z direction for NSM

%% Moment Summation

% Chassis
moment_chassis    = (cross(r_cp1_cm__0 , F_sm1__0) + cross(r_cp2_cm__0 , F_sm2__0) + cross(r_cp3_cm__0 , F_sm3__0) + cross(r_cp4_cm__0 , F_sm4__0));
moment_gryoscopic = (cross(co_omega__0 , JMat__0*co_omega__0));

% NSM - Wheel - Corner 1
moment_wheel_1__1 = -F_cp1__1(1)*r_L_1 + m_d_c/4;

% NSM - Wheel - Corner 2
moment_wheel_2__2 = -F_cp2__2(1)*r_L_2 + m_d_c/4;

% NSM - Wheel - Corner 3
moment_wheel_3__3 = -F_cp3__3(1)*r_L_3 + m_d_c/4;

% NSM - Wheel - Corner 4
moment_wheel_4__4 = -F_cp4__4(1)*r_L_4 + m_d_c/4;

%% Force & Moment Vector Formulation

f_qd_q_u = [F_sm1__0 + F_sm2__0 + F_sm3__0 + F_sm4__0 + F_sm_g__0;
            moment_chassis - moment_gryoscopic;
            F_nsm1__0_3;
            F_nsm2__0_3;
            F_nsm3__0_3;
            F_nsm4__0_3;
            moment_wheel_1__1;
            moment_wheel_2__2;
            moment_wheel_3__3;
            moment_wheel_4__4];

% Generalized velocity vector
q_d  = [x_d y_d z_d TaitBryantAngle_d__0' z_d_1 z_d_2 z_d_3 z_d_4 omega_1 omega_2 omega_3 omega_4]';

%% First-order state-vector formulation
Qdot = [q_d;
        M_Mat__0\f_qd_q_u];


M = M_Mat__0;

O = [u;
     v;
     zs_1;
     zs_2;
     zs_3;
     zs_4;
     zs_dot_1;
     zs_dot_2;
     zs_dot_3;
     zs_dot_4;
     Delta_s_1;
     Delta_s_2;
     Delta_s_3;
     Delta_s_4;
     Delta_d_s_1;
     Delta_d_s_2;
     Delta_d_s_3;
     Delta_d_s_4
     ];







end