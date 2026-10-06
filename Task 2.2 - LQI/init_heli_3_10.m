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


%%%%%%%%%%% Reference Switch 
% 1 = Joystick, 2 = Pulse, 3 = Const (0)
sw_pc = 2;
sw_ecdot = 2; 


%%%%%%%%%%% Pulse values
P = 0.175;    % Value
t1 = 20;       % Start time
t2 = 23;       % End time

t3 = 24;
t4 = 26;


%%%%%%%%%%% Physical constants
g = 9.81; % gravitational constant [m/s^2]
l_c = 0.46; % distance elevation axis to counterweight [m]
l_h = 0.66; % distance elevation axis to helicopter head [m]
l_p = 0.175; % distance pitch axis to motor [m]
m_c = 1.92; % Counterweight mass [kg]
m_p = 0.72; % Motor mass [kg]


%%%%%%%%%%% Moments of inertia [kg m^2]
J_p     = 2 * m_p * l_p^2;                         % pitch axis
J_e     = m_c * l_c^2 + 2*m_p*l_h^2;               % elevation axis
J_lamda = m_c * l_c^2 + 2*m_p * (l_h^2 + l_p^2);   % travel axis


%%%%%%%%%%% Equilibrium constants (measured)
Vs_0 = 7;              % equilibrium sum voltage [V] (measured, Task 2)
e_0 = 0.528;           % elevation offset [rad] (measured, Task 2)


%%%%%%%%%%% Linearization constants (derived)
L_2 = g * (m_c * l_c - 2 * m_p * l_h);   % [N m]
K_f  = -L_2 / (l_h * Vs_0);              % motor force constant [N/V]
L_1 = K_f * l_p;                         % [N m/V]
L_3 = l_h * K_f;

K_1 = L_1 / J_p;                        
K_2 = L_3 / J_e;


%%%%%%%%%%% IQR
A = [0, 1, 0, 0, 0;
     0, 0, 0, 0, 0; 
     0, 0, 0, 0, 0;
    -1, 0, 0, 0, 0;
     0, 0, -1, 0, 0];
 
B = [0, 0; 
     0, K_1; 
     K_2, 0;
     0, 0;
     0, 0];
 
C_ctrl = ctrb(A, B);

if rank(C_ctrl) < size(A, 1)
    error('The linearized helicopter model is not controllable.');
end


pole_mode = 0;

if pole_mode == 1
    disp('SELF CHOSEN POLES MODE')
    poles_des = [1, 3, 3, 2, 5];
    K = place(A, B, poles_des);
    poles = eig(A - B*K);
    
else
    Q = diag([1, 1, 1, 1, 1]);  % [p, p_dot, e_dot, gamma, zeta_LQI]
    R = diag([1, 1]);                % [Vs_thilde, Vd]
    [K, S, poles] = lqr(A, B, Q, R);
end
disp('poles:')
disp(poles)

F = [K(1,1), K(1,3); K(2,1), K(2,3)];


%%%%%%% file shit
data_dir = fullfile(fileparts(mfilename('fullpath')), 'data');

if ~exist(data_dir, 'dir')
    mkdir(data_dir);
end

test_id = 'T1_Normal';
model = 'heli_q8';

data_filename = fullfile(data_dir, [test_id '.mat']);
reg_filename  = fullfile(data_dir, [test_id '_regulatorverdier.mat']);

load_system(model);
set_param([model '/To File'], 'Filename', data_filename);
set_param([model '/To File'], 'MatrixName', 'heli_log');

fprintf('\nData will be logged to:\n    %s\n', data_filename);
fprintf('Regulator values already saved to:\n    %s\n\n', reg_filename);

if pole_mode == 1
    save(reg_filename, 'K', 'F', 'poles', 'P', 't1', 't2', 't3', 't4');
else
    save(reg_filename, 'Q', 'R', 'K', 'F', 'poles', 'P', 't1', 't2');
end
