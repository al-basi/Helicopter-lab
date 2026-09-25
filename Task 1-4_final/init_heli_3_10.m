% FOR HELICOPTER NR 3-10
% This file contains the initialization for the helicopter assignment in
% the course TTK4115. Run this file before you execute QuaRC_ -> Build 
% to build the file heli_q8.mdl.

% Oppdatert h�sten 2006 av Jostein Bakkeheim
% Oppdatert h�sten 2008 av Arnfinn Aas Eielsen
% Oppdatert h�sten 2009 av Jonathan Ronen
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


%%%%%%%%%%% Moments of inertia [kg m^2]
J_p     = 2 * m_p * l_p^2;                        % pitch axis
J_e     = m_c * l_c^2 + 2*m_p*l_h^2;               % elevation axis
J_lamda = m_c * l_c^2 + 2*m_p * (l_h^2 + l_p^2);   % travel axis

%%%%%%%%%%% Equilibrium and linearization constants
V_s0 = 7;                      % equilibrium sum voltage [V] (measured, Task 2)
e_0 = 0.528;                   % elevation offset [rad] (measured, Task 2)
p_0 = -0.122;                  % pitch offset, to minimize travel (measured)
L_2  = g * (m_c * l_c - 2 * m_p * l_h);   % [N m]
K_f  = -L_2 / (l_h * V_s0);    % motor force constant [N/V]

L_1 = K_f * l_p;    % [N m/V]
K_1 = L_1 / J_p;    % pitch loop gain

%%%%%%%%%%% PD koeffisient -- CHANGE test_id, zeta, omega_0 before every test
% test_id is the ONE thing you change. The "To File" block's Filename
% field is set to the (unquoted) variable name "data_filename", so it
% re-reads this value from the base workspace every time you build, no
% manual copy-paste into the block dialog needed anymore.

test_id = 'C3_joystick_pulse';

zeta    = 0.7;
omega_0 = 2.5;

p_step = -0.122;    %C2_stort_step was -0.401 (23 deg)

K_pd = 2*zeta * omega_0 / K_1;
K_pp = omega_0^2 / K_1;

poles = roots([1, 2*zeta * omega_0, omega_0^2]);
disp(poles);

data_dir = fullfile(fileparts(mfilename('fullpath')), 'data');

data_filename = fullfile(data_dir, [test_id '.mat']);
reg_filename  = fullfile(data_dir, [test_id '_regulatorverdier.mat']);

save(reg_filename, 'zeta', 'omega_0', 'K_1', 'K_pp', 'K_pd', 'poles');

fprintf('\nData will be logged to:\n    %s\n', data_filename);
fprintf('Regulator values already saved to:\n    %s\n\n', reg_filename);
