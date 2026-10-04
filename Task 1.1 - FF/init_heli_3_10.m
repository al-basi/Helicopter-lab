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

%%%%%%%%%%% Reference Switch 
% 1 = Joystick, 2 = Pulse, 3 = Const (0)
sw_Vd = 1;
sw_Vs = 1;

%%%%%%%%%%% Gain 
Vd_ff_gain = 7;

%%%%%%%%%%% Pulse values
P = -0.175;    % Value
t1 = 25;       % Start time
t2 = 30;       % End time


%%%%%%% file shit
test_id = 'placeholder';
model = 'heli_q8';

data_dir = fullfile(fileparts(mfilename('fullpath')), 'data');
if ~exist(data_dir, 'dir')
    mkdir(data_dir);
end

data_filename = fullfile(data_dir, [test_id '.mat']);

load_system(model);
set_param([model '/To File'], 'Filename', data_filename);
set_param([model '/To File'], 'MatrixName', 'heli_log');

fprintf('\nData will be logged to:\n    %s\n', data_filename);

