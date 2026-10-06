%% build_observer_model.m
% Bygger observer-delen i heli_q8.slx (Task 3.3) automatisk.
%
% Bruk (fra mappen "Task 3.3 - Observer"):
%   1) Kjor init_heli_3_10.m    (lager A_est, B_est, C_est, L_obs, x0_obs, sw_est ...)
%   2) Kjor dette scriptet      (legger til 3 subsystemer + switch i State Assembly)
%   3) Kjor init_heli_3_10.m igjen (setter filnavn paa To File-blokkene), og Build
%
% Scriptet bygger paa en REN heli_q8.slx. Finnes subsystemene fra for, stopper det.
%
% Nye blokker paa toppnivaa:
%   IMU measurements          (grå)      ingen innganger. Ut: y_full(5), accel(3), gyro(3), new_data
%   Luenberger Observer       (lysblå)   Inn: u(2)=[Vs_tilde;Vd], y_full(5). Ut: x_hat(5), nu
%   Observer Compare and Log  (lysblå)   Scope (x_hat mot encoder) + To File obs_log
% Endring inni State Assembly:
%   Selector/Switch (sw_est) velger om p, p_dot, e_dot kommer fra encoder eller x_hat

mdl  = 'heli_q8';
here = fileparts(mfilename('fullpath'));
if ~isempty(here), cd(here); end

if ~exist('x0_obs','var') || ~exist('C_est','var')
    error('Kjor init_heli_3_10.m forst (den lager C_est, L_obs, x0_obs ...).');
end

load_system(mdl);
load_system('IMU_heli_3_10');

names = {'IMU measurements','Luenberger Observer','Observer Compare and Log'};
for k = 1:numel(names)
    if ~isempty(find_system(mdl,'SearchDepth',1,'Name',names{k}))
        error('"%s" finnes allerede i %s. Bruk en ren heli_q8.slx.', names{k}, mdl);
    end
end

IN1  = 'simulink/Sources/In1';
OUT1 = 'simulink/Sinks/Out1';

%% 1) IMU measurements -----------------------------------------------------
ims = [mdl '/IMU measurements'];
newsub(ims, 300, 520, 150, 110, 'gray');

addb('IMU_heli_3_10/IMU', [ims '/IMU'], 40, 60, 130, 130);
addb('simulink/Sinks/Terminator', [ims '/T_mag'],  220, 205, 20, 20);
addb('simulink/Sinks/Terminator', [ims '/T_temp'], 220, 235, 20, 20);

% Gyro-bias (sett gyro_bias fra en hvile-logg, ellers null)
addb('simulink/Sources/Constant', [ims '/gyro_bias'], 220, 330, 70, 25, 'Value', 'gyro_bias');
addb('simulink/Math Operations/Sum', [ims '/Sum bias'], 330, 285, 30, 40, 'Inputs', '+-');

% Vinkler fra aksellerometer: (2.19) p = atan(ay/az), e = atan(ax/sqrt(ay^2+az^2))
use_mfun = true;
try
    addb('simulink/User-Defined Functions/MATLAB Function', [ims '/Angles'], 240, 60, 110, 50);
    rt = sfroot;
    ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', [ims '/Angles']);
    ch.Script = fileread(fullfile(here, 'Angles.m'));
catch err
    warning('MATLAB Function-blokk feilet (%s). Bruker Fcn-blokker i stedet.', err.message);
    use_mfun = false;
    try, delete_block([ims '/Angles']); catch, end
    addb('simulink/User-Defined Functions/Fcn', [ims '/p_acc'], 240, 50, 120, 30, ...
         'Expr', 'atan(u(2)/u(3))');
    addb('simulink/User-Defined Functions/Fcn', [ims '/e_acc'], 240, 100, 120, 30, ...
         'Expr', 'atan(u(1)/sqrt(u(2)^2+u(3)^2))');
    addb('simulink/Signal Routing/Mux', [ims '/Angles'], 400, 60, 8, 60, 'Inputs', '2');
end

addb('IMU_heli_3_10/Gyro vector to [pitch rate, elevation rate, travle rate]', ...
     [ims '/Gyro vector to euler rates'], 470, 170, 170, 70);
addb('simulink/Signal Routing/Demux', [ims '/angles_demux'], 470, 60, 6, 50, 'Outputs', '2');
addb('simulink/Signal Routing/Demux', [ims '/rates_demux'],  690, 170, 6, 70, 'Outputs', '3');
addb('simulink/Signal Routing/Mux',   [ims '/y_full_mux'],   780, 60, 8, 150, 'Inputs', '5');

addb('simulink/Signal Attributes/Data Type Conversion', [ims '/to double'], 220, 400, 70, 30, ...
     'OutDataTypeStr', 'double');

addb(OUT1, [ims '/y_full'],   880, 128, 30, 14, 'Port', '1');
addb(OUT1, [ims '/accel'],    880, 360, 30, 14, 'Port', '2');
addb(OUT1, [ims '/gyro'],     880, 400, 30, 14, 'Port', '3');
addb(OUT1, [ims '/new_data'], 880, 440, 30, 14, 'Port', '4');

% Linjer
if use_mfun
    add_line(ims, 'IMU/1', 'Angles/1', 'autorouting', 'on');
else
    add_line(ims, 'IMU/1', 'p_acc/1', 'autorouting', 'on');
    add_line(ims, 'IMU/1', 'e_acc/1', 'autorouting', 'on');
    add_line(ims, 'p_acc/1', 'Angles/1', 'autorouting', 'on');
    add_line(ims, 'e_acc/1', 'Angles/2', 'autorouting', 'on');
end
add_line(ims, 'Angles/1', 'angles_demux/1', 'autorouting', 'on');
add_line(ims, 'Angles/1', 'Gyro vector to euler rates/2', 'autorouting', 'on');
add_line(ims, 'IMU/2', 'Sum bias/1', 'autorouting', 'on');
add_line(ims, 'gyro_bias/1', 'Sum bias/2', 'autorouting', 'on');
add_line(ims, 'Sum bias/1', 'Gyro vector to euler rates/1', 'autorouting', 'on');
add_line(ims, 'Gyro vector to euler rates/1', 'rates_demux/1', 'autorouting', 'on');

add_line(ims, 'angles_demux/1', 'y_full_mux/1', 'autorouting', 'on');   % p
add_line(ims, 'rates_demux/1',  'y_full_mux/2', 'autorouting', 'on');   % p_dot
add_line(ims, 'angles_demux/2', 'y_full_mux/3', 'autorouting', 'on');   % e
add_line(ims, 'rates_demux/2',  'y_full_mux/4', 'autorouting', 'on');   % e_dot
add_line(ims, 'rates_demux/3',  'y_full_mux/5', 'autorouting', 'on');   % lambda_dot
add_line(ims, 'y_full_mux/1', 'y_full/1', 'autorouting', 'on');

add_line(ims, 'IMU/1', 'accel/1', 'autorouting', 'on');
add_line(ims, 'IMU/2', 'gyro/1',  'autorouting', 'on');
add_line(ims, 'IMU/3', 'T_mag/1',  'autorouting', 'on');
add_line(ims, 'IMU/4', 'T_temp/1', 'autorouting', 'on');
add_line(ims, 'IMU/5', 'to double/1', 'autorouting', 'on');
add_line(ims, 'to double/1', 'new_data/1', 'autorouting', 'on');

%% 2) Luenberger Observer --------------------------------------------------
%   y   = C_est*y_full           (utvalgte maalinger)
%   yh  = C_est*x_hat            (prediksjon)
%   nu  = y - yh                 (innovasjon)
%   xdot = A_est*x_hat + B_est*u + L_obs*nu
%   x_hat = integral(xdot), x_hat(0) = x0_obs
lob = [mdl '/Luenberger Observer'];
newsub(lob, 520, 520, 150, 110, 'lightBlue');

addb(IN1,  [lob '/u'],      30, 192, 30, 14, 'Port', '1');
addb(IN1,  [lob '/y_full'], 30,  62, 30, 14, 'Port', '2');

addgain(lob, 'Gain C_sel',  'C_est', 110,  48);
addb('simulink/Math Operations/Sum', [lob '/Sum nu'], 250, 60, 30, 40, 'Inputs', '+-');
addgain(lob, 'Gain L_obs',  'L_obs', 330,  50);
addgain(lob, 'Gain B_est',  'B_est', 110, 180);
addb('simulink/Math Operations/Sum', [lob '/Sum xdot'], 500, 130, 30, 70, 'Inputs', '+++');
addb('simulink/Continuous/Integrator', [lob '/Integrator'], 580, 150, 40, 30, ...
     'InitialCondition', 'x0_obs');
addgain(lob, 'Gain A_est',  'A_est', 420, 260, 'left');
addgain(lob, 'Gain C_pred', 'C_est', 250, 150, 'left');

addb(OUT1, [lob '/x_hat'], 720, 158, 30, 14, 'Port', '1');
addb(OUT1, [lob '/nu'],    330,  10, 30, 14, 'Port', '2');

add_line(lob, 'y_full/1',     'Gain C_sel/1',  'autorouting', 'on');
add_line(lob, 'Gain C_sel/1', 'Sum nu/1',      'autorouting', 'on');
add_line(lob, 'Gain C_pred/1','Sum nu/2',      'autorouting', 'on');
add_line(lob, 'Sum nu/1',     'Gain L_obs/1',  'autorouting', 'on');
add_line(lob, 'Sum nu/1',     'nu/1',          'autorouting', 'on');
add_line(lob, 'u/1',          'Gain B_est/1',  'autorouting', 'on');
add_line(lob, 'Gain B_est/1', 'Sum xdot/1',    'autorouting', 'on');   % B*u
add_line(lob, 'Gain L_obs/1', 'Sum xdot/2',    'autorouting', 'on');   % L*nu
add_line(lob, 'Gain A_est/1', 'Sum xdot/3',    'autorouting', 'on');   % A*x_hat
add_line(lob, 'Sum xdot/1',   'Integrator/1',  'autorouting', 'on');
add_line(lob, 'Integrator/1', 'x_hat/1',       'autorouting', 'on');
add_line(lob, 'Integrator/1', 'Gain A_est/1',  'autorouting', 'on');
add_line(lob, 'Integrator/1', 'Gain C_pred/1', 'autorouting', 'on');

%% 3) Observer Compare and Log --------------------------------------------
% Inn: 1 x_hat(5), 2 States(6, fra root Mux), 3 y_full(5), 4 accel(3), 5 gyro(3), 6 new_data
% obs_log.mat: rad 1 tid; 2-6 x_hat [p pd e ed lamd]; 7-11 encoder [p pd e-e0 ed lamd];
%              12-16 y_full; 17-19 accel; 20-22 gyro (raa); 23 New data
cl = [mdl '/Observer Compare and Log'];
newsub(cl, 760, 520, 170, 130, 'lightBlue');

inn = {'x_hat','States','y_full','accel','gyro','new_data'};
for k = 1:numel(inn)
    addb(IN1, [cl '/' inn{k}], 30, 30 + 45*(k-1), 30, 14, 'Port', num2str(k));
end

addb('simulink/Signal Routing/Selector', [cl '/enc_sel'], 140, 85, 70, 30, ...
     'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
     'IndexOptionArray', {'Index vector (dialog)'}, ...
     'IndexParamArray', {'[3 4 5 6 2]'}, 'InputPortWidth', '6');
addb('simulink/Signal Routing/Demux', [cl '/xh_demux'],  260,  20, 6, 120, 'Outputs', '5');
addb('simulink/Signal Routing/Demux', [cl '/enc_demux'], 260, 160, 6, 120, 'Outputs', '5');
for k = 1:5
    addb('simulink/Signal Routing/Mux', [cl sprintf('/pair%d', k)], 340, 20 + 55*(k-1), 6, 40, 'Inputs', '2');
end
addb('simulink/Sinks/Scope', [cl '/Observer vs Encoder'], 440, 20, 60, 260, 'NumInputPorts', '5');
addb('simulink/Signal Routing/Mux', [cl '/log_mux'], 340, 330, 8, 150, 'Inputs', '6');
addb('simulink/Sinks/To File', [cl '/To File obs'], 440, 380, 90, 40, ...
     'Filename', 'obs_log.mat', 'MatrixName', 'obs_log');

add_line(cl, 'States/1',  'enc_sel/1',  'autorouting', 'on');
add_line(cl, 'x_hat/1',   'xh_demux/1', 'autorouting', 'on');
add_line(cl, 'enc_sel/1', 'enc_demux/1','autorouting', 'on');
for k = 1:5
    add_line(cl, sprintf('xh_demux/%d', k),  sprintf('pair%d/1', k), 'autorouting', 'on');
    add_line(cl, sprintf('enc_demux/%d', k), sprintf('pair%d/2', k), 'autorouting', 'on');
    add_line(cl, sprintf('pair%d/1', k), sprintf('Observer vs Encoder/%d', k), 'autorouting', 'on');
end
add_line(cl, 'x_hat/1',    'log_mux/1', 'autorouting', 'on');
add_line(cl, 'enc_sel/1',  'log_mux/2', 'autorouting', 'on');
add_line(cl, 'y_full/1',   'log_mux/3', 'autorouting', 'on');
add_line(cl, 'accel/1',    'log_mux/4', 'autorouting', 'on');
add_line(cl, 'gyro/1',     'log_mux/5', 'autorouting', 'on');
add_line(cl, 'new_data/1', 'log_mux/6', 'autorouting', 'on');
add_line(cl, 'log_mux/1',  'To File obs/1', 'autorouting', 'on');

%% 4) State Assembly: switch encoder / observer ----------------------------
% Gammel kobling: Demux(6) paa States gir p (ut 3), p_dot (ut 4), e_dot (ut 6).
% Ny kobling: [p;p_dot;e_dot] fra enten encoder (States[3 4 6]) eller x_hat([1 2 4]).
sa = [mdl '/State Assembly'];

dms = find_system(sa, 'SearchDepth', 1, 'BlockType', 'Demux');
oldDm = '';
for k = 1:numel(dms)
    if strcmp(get_param(dms{k}, 'Outputs'), '6'), oldDm = dms{k}; end
end
if isempty(oldDm)
    error('Fant ikke Demux med 6 utganger i State Assembly. Er modellen endret?');
end

addb(IN1, [sa '/x_hat'], -60, 290, 30, 14, 'Port', '3');
addb('simulink/Signal Routing/Selector', [sa '/sel_enc'], 40, 200, 60, 30, ...
     'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
     'IndexOptionArray', {'Index vector (dialog)'}, ...
     'IndexParamArray', {'[3 4 6]'}, 'InputPortWidth', '6');
addb('simulink/Signal Routing/Selector', [sa '/sel_hat'], 40, 290, 60, 30, ...
     'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
     'IndexOptionArray', {'Index vector (dialog)'}, ...
     'IndexParamArray', {'[1 2 4]'}, 'InputPortWidth', '5');
addb('simulink/Sources/Constant', [sa '/sw_est'], 40, 245, 60, 25, 'Value', 'sw_est');
addb('simulink/Signal Routing/Switch', [sa '/Estimator Switch'], 150, 210, 60, 80, ...
     'Criteria', 'u2 ~= 0', 'BackgroundColor', 'lightBlue');
addb('simulink/Signal Routing/Demux', [sa '/Demux sel'], 250, 210, 6, 60, 'Outputs', '3');

add_line(sa, 'States/1', 'sel_enc/1', 'autorouting', 'on');
add_line(sa, 'x_hat/1',  'sel_hat/1', 'autorouting', 'on');
add_line(sa, 'sel_hat/1', 'Estimator Switch/1', 'autorouting', 'on');   % x_hat  (sw_est ~= 0)
add_line(sa, 'sw_est/1',  'Estimator Switch/2', 'autorouting', 'on');
add_line(sa, 'sel_enc/1', 'Estimator Switch/3', 'autorouting', 'on');   % encoder
add_line(sa, 'Estimator Switch/1', 'Demux sel/1', 'autorouting', 'on');

% Flytt de gamle forbindelsene fra gammel Demux (ut 3,4,6) til ny Demux (ut 1,2,3)
lh  = get_param(oldDm, 'LineHandles');
phs = get_param([sa '/Demux sel'], 'PortHandles');
map = {3, 1, 'p'; 4, 2, 'p_dot'; 6, 3, 'e_dot'};
for r = 1:3
    L = lh.Outport(map{r,1});
    if L == -1, error('Gammel Demux utgang %d er ikke koblet.', map{r,1}); end
    dst = get_param(L, 'DstPortHandle');
    delete_line(L);
    for d = dst(:).'
        nl = add_line(sa, phs.Outport(map{r,2}), d);
        set_param(nl, 'Name', map{r,3});
    end
end
delete_block(oldDm);   % fjerner ogsaa linjen States -> gammel Demux

%% 5) Koble i toppnivaa ----------------------------------------------------
add_line(mdl, 'Actuation/1',            'Luenberger Observer/1', 'autorouting', 'on');  % u = [Vs_tilde; Vd]
add_line(mdl, 'IMU measurements/1',     'Luenberger Observer/2', 'autorouting', 'on');  % y_full
add_line(mdl, 'Luenberger Observer/1',  'State Assembly/3',      'autorouting', 'on');  % x_hat til regulatoren

add_line(mdl, 'Luenberger Observer/1',  'Observer Compare and Log/1', 'autorouting', 'on');
add_line(mdl, 'Mux/1',                  'Observer Compare and Log/2', 'autorouting', 'on');  % States (6)
add_line(mdl, 'IMU measurements/1',     'Observer Compare and Log/3', 'autorouting', 'on');
add_line(mdl, 'IMU measurements/2',     'Observer Compare and Log/4', 'autorouting', 'on');
add_line(mdl, 'IMU measurements/3',     'Observer Compare and Log/5', 'autorouting', 'on');
add_line(mdl, 'IMU measurements/4',     'Observer Compare and Log/6', 'autorouting', 'on');

save_system(mdl);
fprintf('\nFerdig. Observer-delen er lagt inn i %s.slx.\n', mdl);
fprintf('Neste: kjor init_heli_3_10.m igjen, sjekk Signal Dimensions, Build.\n');
fprintf('sw_est = %d  (0 = encoder styrer, 1 = observer styrer)\n\n', sw_est);

%% Hjelpefunksjoner (maa ligge sist i scriptet) ----------------------------
function addb(src, dst, x, y, w, h, varargin)
    add_block(src, dst, 'Position', [x y x+w y+h], varargin{:});
end

function addgain(sys, name, val, x, y, orient)
    if nargin < 6, orient = 'right'; end
    add_block('simulink/Math Operations/Gain', [sys '/' name], ...
        'Position', [x y x+70 y+40], 'Gain', val, ...
        'Multiplication', 'Matrix(K*u)', 'Orientation', orient);
end

function newsub(path, x, y, w, h, color)
    add_block('built-in/SubSystem', path, 'Position', [x y x+w y+h], 'BackgroundColor', color);
    for t = {'Inport', 'Outport'}
        b = find_system(path, 'SearchDepth', 1, 'BlockType', t{1});
        for i = 1:numel(b), delete_block(b{i}); end
    end
end
