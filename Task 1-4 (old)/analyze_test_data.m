% analyze_test_data.m
%
% Sanity-checks and plots all logged Task 1-3 PD-test data (p1t3_test*.mat)
% in the "data" subfolder next to this script.
%
% Column layout was NOT constant across the day -- it was reverse engineered
% from the Mux wiring in heli_q8.slx (system_root.xml / system_160.xml):
%   Mux in 1-6 <- Heli 3D out 1-6 = [travel, travel_rate, pitch, pitch_rate, elevation, elevation_rate]
%   Mux in 7   <- Pitch controller output              = Vd [V]
%   Mux in 8   <- Elevation controller output + Vs0     = Vs [V]
%   Mux in 9   <- joystick/step -> pitch controller ref = pc [rad]
% "To File" prepends a time row, so:
%   10 rows -> [t, travel, travel_rate, pitch, pitch_rate, elevation, elevation_rate, Vd, Vs, pc]
%    9 rows -> same, but pc was never wired into the Mux yet (missing)
% Confirmed numerically: the elevation row hovers near e0 = 0.528 rad, and the
% Vs row hovers near Vs0 = 7 V, in every file that has that many rows.

clear; clc;
data_dir = fullfile(fileparts(mfilename('fullpath')), 'data');
out_dir  = fullfile(fileparts(mfilename('fullpath')), 'analysis_plots_matlab');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end

E0  = 0.528;
VS0 = 7.0;
full_labels = {'t [s]','travel [rad]','travel\_rate [rad/s]','pitch [rad]','pitch\_rate [rad/s]', ...
               'elevation [rad]','elevation\_rate [rad/s]','Vd [V]','Vs [V]','pc [rad]'};

files = dir(fullfile(data_dir, 'p1t3_test*.mat'));
files = files(~contains({files.name}, 'regulatorverdier'));

fprintf('%-8s %5s %8s %8s  %s\n', 'test','rows','dur[s]','dt[s]','flags');

for k = 1:numel(files)
    fname = fullfile(files(k).folder, files(k).name);
    tok = regexp(files(k).name, 'test([\d\-]+)\.mat$', 'tokens');
    tid = tok{1}{1};

    S = load(fname);
    arr = S.TPE;                 % rows = channels, cols = samples
    [nrows, ~] = size(arr);
    t = arr(1,:);
    dt = mean(diff(t));
    dur = t(end) - t(1);

    if nrows == 10
        labels = full_labels;
    elseif nrows == 9
        labels = full_labels(1:9);
    else
        labels = arrayfun(@(i) sprintf('ch%d', i), 0:nrows-1, 'UniformOutput', false);
    end

    flags = {};
    zero_rows = [];
    for i = 2:nrows
        row = arr(i,:);
        if (max(row) - min(row)) < 1e-9 && abs(mean(row)) < 1e-9
            zero_rows(end+1) = i; %#ok<AGROW>
        end
        lab = labels{i};
        if contains(lab, 'elevation [rad]') && abs(mean(row) - E0) > 0.15
            flags{end+1} = sprintf('%s mean=%.3f far from e0=%.3f', lab, mean(row), E0); %#ok<AGROW>
        end
        if contains(lab, 'Vs [V]') && (max(row) > 30 || min(row) < -30)
            flags{end+1} = sprintf('%s out of plausible supply range [%.1f, %.1f]', lab, min(row), max(row)); %#ok<AGROW>
        end
        if contains(lab, 'pitch [rad]') && (max(row) > 1.3 || min(row) < -1.3)
            flags{end+1} = sprintf('%s very large excursion [%.2f, %.2f] (mechanical stop?)', lab, min(row), max(row)); %#ok<AGROW>
        end
    end
    if ~isempty(zero_rows)
        flags{end+1} = ['ALL-ZERO rows (not logged): ' mat2str(zero_rows-1)]; %#ok<AGROW>
    end
    if dur < 10
        flags{end+1} = sprintf('DURATION < 10s (%.1fs)', dur); %#ok<AGROW>
    end

    regfile = fullfile(files(k).folder, sprintf('p1t3_test%s_regulatorverdier.mat', tid));
    reg_str = '';
    if exist(regfile, 'file')
        R = load(regfile);
        if isfield(R,'zeta') && isfield(R,'omega_0')
            reg_str = sprintf('zeta=%.4g, omega0=%.4g', R.zeta, R.omega_0);
        end
    end

    fprintf('%-8s %5d %8.2f %8.4f  %s\n', tid, nrows, dur, dt, strjoin(flags, ' | '));

    % --- plot ---
    n_signal_rows = nrows - 1;
    fig = figure('Visible','off','Position',[100 100 900 150*n_signal_rows]);
    for i = 2:nrows
        ax = subplot(n_signal_rows, 1, i-1);
        plot(t, arr(i,:), 'LineWidth', 0.8); grid on;
        ylabel(labels{i}, 'FontSize', 8);
        if ismember(i, zero_rows)
            text(0.5, 0.5, 'ALL ZERO', 'Color','r', 'Units','normalized', ...
                 'HorizontalAlignment','center');
        end
        if i == 2
            title(sprintf('Task 1-4 test %s (rows=%d, duration=%.1fs)  %s', tid, nrows, dur, reg_str), ...
                  'Interpreter','none', 'FontSize', 10);
        end
    end
    xlabel('time [s]');
    saveas(fig, fullfile(out_dir, sprintf('test%s_raw_channels.png', tid)));
    close(fig);
end

fprintf('\nPlots written to: %s\n', out_dir);
