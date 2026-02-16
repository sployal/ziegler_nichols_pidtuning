%% ZIEGLER-NICHOLS PID TUNING - FREQUENCY RESPONSE METHOD
% Control Engineering - Automatic PID Tuning Demo
clc; clear; close all;

% --- Define Plant ---
s = tf('s');
G = 1/(s*(s+1));  % Second-order plant with integrator

fprintf('============================================\n');
fprintf('ZIEGLER-NICHOLS PID TUNING DEMONSTRATION\n');
fprintf('Using Frequency Response Method\n');
fprintf('============================================\n\n');
fprintf('Plant: G(s) = 1/(s(s+1))\n\n');

%% STEP 1: Find Ultimate Gain (Ku) and Period (Pu) using Frequency Response
fprintf('STEP 1: Finding Ultimate Gain and Period...\n');
fprintf('-------------------------------------------\n');

% Create frequency vector
w = logspace(-2, 2, 1000);

% Get frequency response
[mag, phase, wout] = bode(G, w);
mag = squeeze(mag);
phase = squeeze(phase);

% Find -180° crossover frequency (phase crossover)
idx_180 = find(phase <= -180, 1, 'first');
if isempty(idx_180)
    fprintf('Warning: No -180° crossover found. Using approximation.\n');
    idx_180 = length(phase);
end

w_180 = wout(idx_180);  % Critical frequency
mag_180 = mag(idx_180);  % Magnitude at -180°

% Ultimate gain: Ku = 1/|G(jω)| at -180° phase
Ku = 1 / mag_180;

% Ultimate period: Pu = 2π/ω at -180°
Pu = 2*pi / w_180;

fprintf('  → Critical Frequency ω_180 = %.4f rad/s\n', w_180);
fprintf('  → Ultimate Gain Ku = %.4f\n', Ku);
fprintf('  → Ultimate Period Pu = %.4f seconds\n\n', Pu);

%% STEP 2: Calculate Ziegler-Nichols Tuning Parameters
fprintf('STEP 2: Calculating Z-N Tuning Parameters...\n');
fprintf('-------------------------------------------\n');

% Ziegler-Nichols formulas for different controllers
% P Controller
Kp_P_zn = 0.5 * Ku;
Ki_P_zn = 0;
Kd_P_zn = 0;
fprintf('P Controller:\n');
fprintf('  Kp = %.4f\n\n', Kp_P_zn);

% PI Controller
Kp_PI_zn = 0.45 * Ku;
Ki_PI_zn = 1.2 * Kp_PI_zn / Pu;
Kd_PI_zn = 0;
fprintf('PI Controller:\n');
fprintf('  Kp = %.4f\n', Kp_PI_zn);
fprintf('  Ki = %.4f\n\n', Ki_PI_zn);

% PD Controller (Note: PD is not suitable for plants with integrators)
Kp_PD_zn = 0.8 * Ku;
Ki_PD_zn = 0;
Kd_PD_zn = Kp_PD_zn * Pu / 8;
fprintf('PD Controller:\n');
fprintf('  Kp = %.4f\n', Kp_PD_zn);
fprintf('  Kd = %.4f\n', Kd_PD_zn);
fprintf('  Note: PD unstable for this plant type\n\n');

% PID Controller (Classic Z-N)
Kp_PID_zn = 0.6 * Ku;
Ki_PID_zn = 2 * Kp_PID_zn / Pu;
Kd_PID_zn = Kp_PID_zn * Pu / 8;
fprintf('PID Controller:\n');
fprintf('  Kp = %.4f\n', Kp_PID_zn);
fprintf('  Ki = %.4f\n', Ki_PID_zn);
fprintf('  Kd = %.4f\n\n', Kd_PID_zn);

%% STEP 3: Create Controllers with Z-N Parameters
C_P_zn   = tf(Kp_P_zn, 1);
C_PI_zn  = tf([Kp_PI_zn, Ki_PI_zn], [1, 0]);
C_PD_zn  = tf([Kd_PD_zn, Kp_PD_zn], 1);
C_PID_zn = tf([Kd_PID_zn, Kp_PID_zn, Ki_PID_zn], [1, 0]);

% Closed-loop systems
T_P_zn   = feedback(C_P_zn*G, 1);
T_PI_zn  = feedback(C_PI_zn*G, 1);
T_PD_zn  = feedback(C_PD_zn*G, 1);
T_PID_zn = feedback(C_PID_zn*G, 1);

%% STEP 4: Simulate Step Responses with extended time
fprintf('STEP 3: Simulating Step Responses...\n');
fprintf('-------------------------------------------\n');

t = 0:0.01:40;  % Extended simulation time

% Simulate with options to handle warnings
opt = stepDataOptions('StepAmplitude', 1);

[yP_zn, tP]   = step(T_P_zn, t);
[yPI_zn, tPI]  = step(T_PI_zn, t);
[yPD_zn, tPD]  = step(T_PD_zn, t);
[yPID_zn, tPID] = step(T_PID_zn, t);

%% STEP 5: Calculate Performance Metrics with proper options
% Check stability first
stable_P = isstable(T_P_zn);
stable_PI = isstable(T_PI_zn);
stable_PD = isstable(T_PD_zn);
stable_PID = isstable(T_PID_zn);

fprintf('Stability Analysis:\n');
fprintf('  P Controller:   %s\n', string(stable_P));
fprintf('  PI Controller:  %s\n', string(stable_PI));
fprintf('  PD Controller:  %s\n', string(stable_PD));
fprintf('  PID Controller: %s\n\n', string(stable_PID));

% Calculate stepinfo only for stable systems
if stable_P
    info_P = stepinfo(T_P_zn, 'SettlingTimeThreshold', 0.02);
else
    info_P = struct('RiseTime', NaN, 'SettlingTime', NaN, 'Overshoot', NaN, 'Peak', NaN, 'PeakTime', NaN);
end

if stable_PI
    info_PI = stepinfo(T_PI_zn, 'SettlingTimeThreshold', 0.02);
else
    info_PI = struct('RiseTime', NaN, 'SettlingTime', NaN, 'Overshoot', NaN, 'Peak', NaN, 'PeakTime', NaN);
end

if stable_PD
    info_PD = stepinfo(T_PD_zn, 'SettlingTimeThreshold', 0.02);
else
    info_PD = struct('RiseTime', NaN, 'SettlingTime', NaN, 'Overshoot', NaN, 'Peak', NaN, 'PeakTime', NaN);
end

if stable_PID
    info_PID = stepinfo(T_PID_zn, 'SettlingTimeThreshold', 0.02);
else
    info_PID = struct('RiseTime', NaN, 'SettlingTime', NaN, 'Overshoot', NaN, 'Peak', NaN, 'PeakTime', NaN);
end

%% STEP 6: Create Visualizations
figure('Position', [50 50 1400 900], 'Color', 'w');

% Subplot 1: Step Response Comparison (only stable systems)
subplot(2,3,1);
hold on;
if stable_P, plot(tP, yP_zn, 'r-', 'LineWidth', 2.5); end
if stable_PI, plot(tPI, yPI_zn, 'b-', 'LineWidth', 2.5); end
if stable_PD, plot(tPD, yPD_zn, 'g-', 'LineWidth', 2.5); end
if stable_PID, plot(tPID, yPID_zn, 'k-', 'LineWidth', 2.5); end
yline(1, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5);
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Output', 'FontSize', 11);
title('Step Response Comparison', 'FontSize', 12, 'FontWeight', 'bold');

% Build legend dynamically
legend_entries = {};
if stable_P, legend_entries{end+1} = 'P'; end
if stable_PI, legend_entries{end+1} = 'PI'; end
if stable_PD, legend_entries{end+1} = 'PD'; end
if stable_PID, legend_entries{end+1} = 'PID'; end
legend_entries{end+1} = 'Setpoint';
legend(legend_entries, 'Location', 'southeast');
xlim([0 40]);
ylim([0 1.5]);

% Subplot 2: PID Response Detail
subplot(2,3,2);
if stable_PID
    plot(tPID, yPID_zn, 'k-', 'LineWidth', 3); hold on;
    yline(1, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5);
    yline(1.05, ':', 'Color', [0.8 0.2 0.2], 'LineWidth', 1);
    yline(0.95, ':', 'Color', [0.2 0.2 0.8], 'LineWidth', 1);
    text(35, 1.05, '+5%', 'FontSize', 8);
    text(35, 0.95, '-5%', 'FontSize', 8);
    ylim([0 1.3]);
else
    text(0.5, 0.5, 'PID Unstable', 'HorizontalAlignment', 'center', 'FontSize', 14, 'Color', 'r');
end
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Output', 'FontSize', 11);
title('PID Response (Ziegler-Nichols)', 'FontSize', 12, 'FontWeight', 'bold');
xlim([0 40]);

% Subplot 3: Tuning Parameters Bar Chart
subplot(2,3,3);
params = [Kp_P_zn, Ki_P_zn, Kd_P_zn; 
          Kp_PI_zn, Ki_PI_zn, Kd_PI_zn; 
          Kp_PD_zn, Ki_PD_zn, Kd_PD_zn;
          Kp_PID_zn, Ki_PID_zn, Kd_PID_zn];
b = bar(params, 'grouped');
b(1).FaceColor = [0.2 0.6 0.8];
b(2).FaceColor = [0.8 0.4 0.2];
b(3).FaceColor = [0.4 0.8 0.3];
set(gca, 'XTickLabel', {'P', 'PI', 'PD', 'PID'});
ylabel('Gain Value', 'FontSize', 11);
title('Z-N Tuning Parameters', 'FontSize', 12, 'FontWeight', 'bold');
legend('Kp', 'Ki', 'Kd', 'Location', 'northwest');
grid on;

% Subplot 4: Bode Plot with Critical Points
subplot(2,3,4);
margin(G);
grid on;
title('Bode Diagram with Stability Margins', 'FontSize', 12, 'FontWeight', 'bold');

% Subplot 5: Pole-Zero Map for PID
subplot(2,3,5);
if stable_PID
    pzmap(T_PID_zn);
    title('Pole-Zero Map (PID Closed-Loop)', 'FontSize', 12, 'FontWeight', 'bold');
else
    text(0.5, 0.5, 'PID System Unstable', 'HorizontalAlignment', 'center', 'FontSize', 12, 'Color', 'r');
end
grid on;

% Subplot 6: Performance Metrics Table
subplot(2,3,6);
axis off;

% Create performance table with proper handling of NaN values
text_str = sprintf('PERFORMANCE METRICS\n');
text_str = [text_str sprintf('═════════════════════════════════════\n\n')];
text_str = [text_str sprintf('Tuning Method: Ziegler-Nichols\n')];
text_str = [text_str sprintf('Ultimate Gain:   Ku = %.4f\n', Ku)];
text_str = [text_str sprintf('Ultimate Period: Pu = %.4f s\n', Pu)];
text_str = [text_str sprintf('Critical Freq:   ω  = %.4f rad/s\n\n', w_180)];
text_str = [text_str sprintf('─────────────────────────────────────\n')];
text_str = [text_str sprintf('Controller   RiseTime  Overshoot  Settling\n')];
text_str = [text_str sprintf('─────────────────────────────────────\n')];

% Format rows with stability check
if stable_P && ~isnan(info_P.RiseTime)
    text_str = [text_str sprintf('%-12s %8.3fs  %8.2f%%  %8.3fs\n', 'P', info_P.RiseTime, info_P.Overshoot, info_P.SettlingTime)];
else
    text_str = [text_str sprintf('%-12s    UNSTABLE OR NO STEADY STATE\n', 'P')];
end

if stable_PI && ~isnan(info_PI.RiseTime)
    text_str = [text_str sprintf('%-12s %8.3fs  %8.2f%%  %8.3fs\n', 'PI', info_PI.RiseTime, info_PI.Overshoot, info_PI.SettlingTime)];
else
    text_str = [text_str sprintf('%-12s    UNSTABLE OR NO STEADY STATE\n', 'PI')];
end

if stable_PD && ~isnan(info_PD.RiseTime)
    text_str = [text_str sprintf('%-12s %8.3fs  %8.2f%%  %8.3fs\n', 'PD', info_PD.RiseTime, info_PD.Overshoot, info_PD.SettlingTime)];
else
    text_str = [text_str sprintf('%-12s    UNSTABLE\n', 'PD')];
end

if stable_PID && ~isnan(info_PID.RiseTime)
    text_str = [text_str sprintf('%-12s %8.3fs  %8.2f%%  %8.3fs\n', 'PID', info_PID.RiseTime, info_PID.Overshoot, info_PID.SettlingTime)];
else
    text_str = [text_str sprintf('%-12s    UNSTABLE OR NO STEADY STATE\n', 'PID')];
end

text(0.05, 0.5, text_str, 'FontSize', 9, 'FontName', 'Courier New', ...
     'VerticalAlignment', 'middle', 'FontWeight', 'bold');

sgtitle('Ziegler-Nichols PID Tuning - Complete Analysis', ...
        'FontSize', 15, 'FontWeight', 'bold');

%% STEP 7: Display Detailed Console Output
fprintf('\nSTEP 4: Detailed Performance Analysis\n');
fprintf('═══════════════════════════════════════════════\n');
systems = {'P', 'PI', 'PD', 'PID'};
infos = {info_P, info_PI, info_PD, info_PID};
stables = [stable_P, stable_PI, stable_PD, stable_PID];

for i = 1:4
    fprintf('\n%s Controller Performance:\n', systems{i});
    if stables(i) && ~isnan(infos{i}.RiseTime)
        fprintf('  Rise Time:       %.4f s\n', infos{i}.RiseTime);
        fprintf('  Settling Time:   %.4f s\n', infos{i}.SettlingTime);
        fprintf('  Overshoot:       %.2f %%\n', infos{i}.Overshoot);
        fprintf('  Peak Value:      %.4f\n', infos{i}.Peak);
        fprintf('  Peak Time:       %.4f s\n', infos{i}.PeakTime);
    else
        fprintf('  Status:          UNSTABLE or NO STEADY STATE\n');
    end
end

fprintf('\n═══════════════════════════════════════════════\n');
fprintf('ZIEGLER-NICHOLS TUNING COMPLETE ✓\n');
fprintf('═══════════════════════════════════════════════\n\n');

% Display controller transfer functions
fprintf('CONTROLLER TRANSFER FUNCTIONS:\n');
fprintf('─────────────────────────────────────\n');
fprintf('PID Controller: C(s) = Kp + Ki/s + Kd*s\n');
fprintf('               C(s) = %.4f + %.4f/s + %.4f*s\n\n', Kp_PID_zn, Ki_PID_zn, Kd_PID_zn);

fprintf('Notes:\n');
fprintf('• P controller may not reach steady state (no integral action)\n');
fprintf('• PD controller is unstable for plants with integrators\n');
fprintf('• PI and PID controllers provide good steady-state tracking\n\n');

if stable_PID
    fprintf('Recommendation: The PID controller provides\n');
    fprintf('the best balance of rise time, overshoot,\n');
    fprintf('and steady-state error elimination.\n');
else
    fprintf('Recommendation: The PI controller is the best\n');
    fprintf('option for this plant, providing stability and\n');
    fprintf('zero steady-state error.\n');
end