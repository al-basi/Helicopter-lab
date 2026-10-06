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


%%%%%%%%%%% Estimator switch and IMU (Part III)
% sw_est: 0 = controller uses encoder, 1 =  x_hat from observer
sw_est = 0;
PORT = 9;                  % COM-port 
gyro_bias = zeros(3,1);    % [wx; wy; wz] i rad/s. 


%%%%%%%%%%% Doublet values
% if you only want a Pulse, set t3 and t4 to a very high number
P = 0.175;     % Pulse amplitude
t1 = 20;       % First pulse start time
t2 = 23;       % First pulse end timw

t3 = 24;       % Second pulse start time
t4 = 26;       % Second pulse end time


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
L_4 = l_h * K_f;
K_3 = L_4 * Vs_0 / J_lamda;           % = -L_2 / J_lamda


%%%%%%%%%%% IQR
A_ctrl = [0, 1, 0, 0, 0;
     0, 0, 0, 0, 0; 
     0, 0, 0, 0, 0;
    -1, 0, 0, 0, 0;
     0, 0, -1, 0, 0];
 
B_ctrl = [0, 0; 
     0, K_1; 
     K_2, 0;
     0, 0;
     0, 0];
 
C_ctrl = ctrb(A_ctrl, B_ctrl);

if rank(C_ctrl) < size(A_ctrl, 1)
    error('The linearized helicopter model is not controllable.');
end


pole_mode = 0;

if pole_mode == 1
    disp('SELF CHOSEN POLES MODE')
    poles_des = [1, 3, 3, 2, 5];
    K = place(A_ctrl, B_ctrl, poles_des);
    poles = eig(A_ctrl - B_ctrl*K);
    
else
    Q = diag([1, 1, 1, 1, 1]);       % [p, p_dot, e_dot, gamma, zeta_LQI]
    R = diag([1, 1]);                % [Vs_thilde, Vd]
    [K, S, poles] = lqr(A_ctrl, B_ctrl, Q, R);
end
disp('poles:')
disp(poles)

F = [K(1,1), K(1,3); K(2,1), K(2,3)];


%%%%%%%%%%% Luenberger observer
% x_est = [p, p_dot, e, e_dot, lambda_dot]',  u = [Vs_tilde, Vd]
A_est = [0,   1, 0, 0, 0;
         0,   0, 0, 0, 0;
         0,   0, 0, 1, 0;
         0,   0, 0, 0, 0;
         K_3, 0, 0, 0, 0];

B_est = [0,   0;
         0,   K_1;
         0,   0;
         K_2, 0;
         0,   0];

%%%%%%%%%%% Observability
% Measure e and lambda_dot only
C_est_min = [0, 0, 1, 0, 0;
             0, 0, 0, 0, 1];

O_est_min = obsv(A_est, C_est_min);

if rank(O_est_min) < size(A_est, 1)
    error('The estimator model is not observable with C_est_min.');
end


%%%%%%%%%%% Luenberger observer
% c_mode = 1: All states
% c_mode = 2: minimal states  [e, lambda_dot]
c_mode = 1;

if c_mode == 1
    C_est = eye(5);
else
    C_est = C_est_min;
end

obs_speed = 5;                                  
p_obs = -obs_speed * [1, 1.2, 1.4, 1.6, 1.8];   % desired observer-poles

L_obs = place(A_est', C_est', p_obs)';          
poles_obs = eig(A_est - L_obs*C_est);           % Actual observer-poler
x0_obs = [0; 0; -e_0; 0; 0];                    % Starts from the ground 
                                                % e = -e_0

disp('observer poles:')
disp(poles_obs)


%%%%%%% file shit
data_dir = fullfile(fileparts(mfilename('fullpath')), 'data');

if ~exist(data_dir, 'dir')
    mkdir(data_dir);
end

test_id = 'T1_Normal';
model = 'heli_q8';

data_filename = fullfile(data_dir, [test_id '.mat']);
reg_filename = fullfile(data_dir, [test_id '_regulatorverdier.mat']);
obs_filename = fullfile(data_dir, [test_id '_obs.mat']);
load_system(model);

set_param([model '/To File'], 'Filename', data_filename);
set_param([model '/To File'], 'MatrixName', 'heli_log');

set_param([model '/To File obs'], 'Filename', obs_filename);
set_param([model '/To File obs'], 'MatrixName', 'obs_log');

if pole_mode == 1
    save(reg_filename, 'K', 'F', 'poles', 'P', 't1', 't2', 't3', 't4');
else
    save(reg_filename, 'Q', 'R', 'K', 'F', 'poles', 'P', 't1', 't2');
end
save(reg_filename, 'C_est', 'L_obs', 'poles_obs', 'p_obs', 'obs_speed', 'c_mode', 'sw_est', '-append');
