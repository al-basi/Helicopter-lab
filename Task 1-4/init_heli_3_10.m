% FOR HELICOPTER NR 3-10
% This file contains the initialization for the helicopter assignment in
% the course TTK4115. Run this file before you execute QuaRC_ -> Build 
% to build the file heli_q8.mdl.

% Oppdatert høsten 2006 av Jostein Bakkeheim
% Oppdatert høsten 2008 av Arnfinn Aas Eielsen
% Oppdatert høsten 2009 av Jonathan Ronen
% Updated fall 2010, Dominik Breu
% Updated fall 2013, Mark Haring
% Updated spring 2015, Mark Haring


%%%%%%%%%%% Calibration of the encoder and the hardware for the specific
%%%%%%%%%%% helicopter
Joystick_gain_x = 1;
Joystick_gain_y = -1;


%%%%%%%%%%% Physical constants
g = 9.81; % gravitational constant [m/s^2]
l_c = 0.46; % distance elevation axis to counterweight [m]
l_h = 0.66; % distance elevation axis to helicopter head [m]
l_p = 0.175; % distance pitch axis to motor [m]
m_c = 1.92; % Counterweight mass [kg]
m_p = 0.72; % Motor mass [kg]

%%%%%%%%%%% Custom constants
e_0 = 0.528; % elevation offset


J_p = 2 * m_p * l_p^2;
J_e = m_c * l_c^2 + 2*m_p*l_h^2;
J_lamda = m_c * l_c ^2 + 2*m_p * (l_h^2 + l_p^2);

L_2 = g * (m_c * l_c - 2 * m_p * l_h);
V_s0 = 7;
K_f = -L_2 / (l_h * V_s0);

L_1 = K_f * l_p;
K_1 = L_1 / J_p;

%%%%%%%%%%% PD koeffisient
zeta = 0.001;
omega_0 = 0.01;

%Ts= 2;
%omega_0 = 4/(zeta*Ts);


K_pd = 2*zeta * omega_0 / K_1;
K_pp = omega_0^2 /K_1;

poles =  roots([1, 2*zeta * omega_0, omega_0^2]);
disp(poles);

filnavn = 'p1t3_test12_regulatorverdier.mat';
save(filnavn, 'zeta', 'omega_0', 'K_1', 'K_pp', 'K_pd', 'poles');
