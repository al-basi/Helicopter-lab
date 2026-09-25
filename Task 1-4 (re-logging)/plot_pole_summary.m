% plot_pole_summary.m
%
% Samler ALLE *_regulatorverdier.mat-filer i "data"-mappa og plotter
% polplasseringene sammen, med test_id (fra filnavnet) som etikett.
%
% Ingen egen "master"-fil å vedlikeholde: kjør dette scriptet på nytt
% hver gang dere har logget flere tester, det bygger oversikten fra
% filene på disk hver gang, så det kan aldri komme ut av sync.

clear; clc;
data_dir = fullfile(fileparts(mfilename('fullpath')), 'data');

files = dir(fullfile(data_dir, '*_regulatorverdier.mat'));
if isempty(files)
    error('Fant ingen *_regulatorverdier.mat i %s', data_dir);
end

n = numel(files);
test_id = strings(n,1);
zeta    = nan(n,1);
omega_0 = nan(n,1);
K_1     = nan(n,1);
K_pp    = nan(n,1);
K_pd    = nan(n,1);
poles   = cell(n,1);

for k = 1:n
    S = load(fullfile(files(k).folder, files(k).name));
    test_id(k) = erase(files(k).name, '_regulatorverdier.mat');
    zeta(k)    = S.zeta;
    omega_0(k) = S.omega_0;
    K_1(k)     = S.K_1;
    K_pp(k)    = S.K_pp;
    K_pd(k)    = S.K_pd;
    poles{k}   = S.poles;
end

% Valgfritt: bygg en tabell for rask oversikt / kopiere inn i rapporten.
% Denne skrives på nytt hver gang (ikke append), så den kan aldri drifte.
T = table(test_id, zeta, omega_0, K_1, K_pp, K_pd, poles);
disp(T);
writetable(T(:,1:6), fullfile(data_dir, 'test_summary.csv')); % poles droppes (komplekse tall i csv er stygt)

% --- Polplott ---
figure('Name','Pol-oversikt, alle logget tester');
hold on; grid on; axis equal;
xline(0, 'k-', 'LineWidth', 1);
yline(0, 'k-', 'LineWidth', 1);
colors = lines(n);
for k = 1:n
    p = poles{k};
    plot(real(p), imag(p), 'o', 'MarkerSize', 9, 'MarkerFaceColor', colors(k,:), ...
         'MarkerEdgeColor', 'k');
    [~, idx] = max(imag(p));
    text(real(p(idx))+0.15, imag(p(idx))+0.15, ...
         sprintf('%s (\\zeta=%.2g, \\omega_0=%.2g)', test_id(k), zeta(k), omega_0(k)), ...
         'FontSize', 8, 'Color', colors(k,:), 'Interpreter', 'tex');
end
xlabel('Re(s)'); ylabel('Im(s)');
title('Faktiske polplasseringer for logget tester');
