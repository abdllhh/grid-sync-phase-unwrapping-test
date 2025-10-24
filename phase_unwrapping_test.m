%% Grid Synchronization Phase Unwrapping Test Suite
% This script tests phase unwrapping methods and various grid disturbances

%% Test Configuration
TEST_CASE = 4; % Change this to test different scenarios:
               % 1 = Normal operation (baseline)
               % 2 = 30-degree phase jump at t=0.1s
               % 3 = Frequency jump (50Hz to 51Hz)
               % 4 = Comparison: WITH vs WITHOUT unwrapping

%% Common Parameters
Ts = 5e-5;              % 50 microseconds (20kHz sampling)
M_PI = pi;
simulation_time = 0.4;  % 400ms simulation
N_samples = simulation_time / Ts;

% Controller gains
wKp = 0.001;
wKp1 = 0.001;

%% Initialize Arrays for Plotting
time_array = zeros(1, N_samples);
grid_voltage = zeros(1, N_samples);
inv_voltage = zeros(1, N_samples);
grid_angle = zeros(1, N_samples);
inv_angle = zeros(1, N_samples);
delta_w_array = zeros(1, N_samples);
phase_error_array = zeros(1, N_samples);
frequency_array = zeros(1, N_samples);

%% Run Selected Test Case
switch TEST_CASE
    case 1
        fprintf('Running Test Case 1: Normal Operation (Baseline)\n');
        run_normal_operation();
    case 2
        fprintf('Running Test Case 2: 30-Degree Phase Jump at t=0.1s\n');
        run_phase_jump_test();
    case 3
        fprintf('Running Test Case 3: Frequency Jump (50Hz to 51Hz)\n');
        run_frequency_jump_test();
    case 4
        fprintf('Running Test Case 4: Comparison WITH vs WITHOUT Unwrapping\n');
        run_comparison_test();
end

%% TEST CASE 1: Normal Operation (Baseline)
function run_normal_operation()
    Ts = 5e-5;
    M_PI = pi;
    t1 = 0;
    WTinv_update = pi/2;
    
    wKp = 0.001;
    wKp1 = 0.001;
    
    grid_wt_m = pi/2;
    grid_wt_error = 0;
    delta_w_sync = 0;
    wt_error_prev = 0;
    delta_w = 0;
    
    N = 8000;
    time = zeros(1, N);
    y9 = zeros(1, N);
    y10 = zeros(1, N);
    delta_w_plot = zeros(1, N);
    freq_plot = zeros(1, N);
    phase_error = zeros(1, N);
    grid_angle_plot = zeros(1, N);
    inv_angle_plot = zeros(1, N);
    
    figure('Position', [100, 100, 1400, 800]);
    
    for a = 1:N
        time(a) = a * Ts;
        
        % Grid simulation with unwrapping
        grid_wt_m = grid_wt_m - 0.015707963267949;
        grid_wt_m = atan2(sin(grid_wt_m), cos(grid_wt_m)); % Unwrap
        y9(a) = 1 * cos(grid_wt_m);
        grid_angle_plot(a) = grid_wt_m;
        
        % Inverter control
        if (a <= 400)
            % Phase 1: Initial tracking
            theta_ref = (-2 * M_PI * 50 * t1) + pi/2;
            theta_ref = atan2(sin(theta_ref), cos(theta_ref)); % Unwrap
            
            y10(a) = 1 * cos(theta_ref);
            
            wt_prev = WTinv_update;
            WTinv_update = theta_ref;
            delta_w_raw = WTinv_update - wt_prev;
            
            % Handle delta_w wrapping
            if delta_w_raw > pi
                delta_w = delta_w_raw - 2*pi;
            elseif delta_w_raw < -pi
                delta_w = delta_w_raw + 2*pi;
            else
                delta_w = delta_w_raw;
            end
            
            t1 = t1 + Ts;
            
        elseif (a > 400) && (a <= 800)
            % Phase 2: Free running
            WTinv_update = WTinv_update + delta_w;
            WTinv_update = atan2(sin(WTinv_update), cos(WTinv_update));
            y10(a) = 1 * cos(WTinv_update);
            
        elseif (a > 800) && (a <= 3000)
            % Phase 3: Synchronization
            WTinv_update = WTinv_update + delta_w;
            
            grid_wt_error = sin(grid_wt_m - WTinv_update);
            delta_w_sync = wKp * grid_wt_error + wKp1 * (grid_wt_error - wt_error_prev);
            wt_error_prev = grid_wt_error;
            WTinv_update = WTinv_update + delta_w_sync;
            
            WTinv_update = atan2(sin(WTinv_update), cos(WTinv_update));
            y10(a) = 1 * cos(WTinv_update);
            phase_error(a) = grid_wt_error;
            
        elseif (a > 3000)
            % Phase 4: Free running again
            WTinv_update = WTinv_update + delta_w;
            WTinv_update = atan2(sin(WTinv_update), cos(WTinv_update));
            y10(a) = 1 * cos(WTinv_update);
        end
        
        inv_angle_plot(a) = WTinv_update;
        delta_w_plot(a) = delta_w;
        freq_plot(a) = delta_w / (2*pi*Ts); % Convert to Hz
    end
    
    % Plotting
    subplot(4,1,1);
    plot(time*1000, y9, 'r-', 'LineWidth', 1.5); hold on;
    plot(time*1000, y10, 'b--', 'LineWidth', 1.5);
    xlabel('Time (ms)'); ylabel('Voltage (pu)');
    title('Grid vs Inverter Voltage Waveforms');
    legend('Grid', 'Inverter');
    grid on;
    xlim([0 max(time)*1000]);
    
    subplot(4,1,2);
    plot(time*1000, grid_angle_plot, 'r-', 'LineWidth', 1.5); hold on;
    plot(time*1000, inv_angle_plot, 'b--', 'LineWidth', 1.5);
    xlabel('Time (ms)'); ylabel('Angle (rad)');
    title('Phase Angles (Unwrapped to [-π, π])');
    legend('Grid', 'Inverter');
    grid on;
    xlim([0 max(time)*1000]);
    
    subplot(4,1,3);
    plot(time*1000, delta_w_plot, 'g-', 'LineWidth', 1.5);
    xlabel('Time (ms)'); ylabel('delta\_w (rad/sample)');
    title('Angular Velocity per Sample');
    grid on;
    xlim([0 max(time)*1000]);
    yline(-0.015708, 'r--', '50Hz reference');
    
    subplot(4,1,4);
    plot(time*1000, freq_plot, 'm-', 'LineWidth', 1.5);
    xlabel('Time (ms)'); ylabel('Frequency (Hz)');
    title('Estimated Frequency');
    grid on;
    xlim([0 max(time)*1000]);
    yline(50, 'r--', '50Hz reference');
    ylim([49 51]);
    
    fprintf('Test Complete: delta_w stable = %.6f rad/sample (%.2f Hz)\n', ...
            mean(delta_w_plot(100:end)), mean(abs(freq_plot(100:end))));
end

%% TEST CASE 2: Phase Jump Test
function run_phase_jump_test()
    Ts = 5e-5;
    M_PI = pi;
    t1 = 0;
    WTinv_update = pi/2;
    
    wKp = 0.001;
    wKp1 = 0.001;
    
    grid_wt_m = pi/2;
    grid_wt_error = 0;
    delta_w_sync = 0;
    wt_error_prev = 0;
    delta_w = 0;
    
    phase_jump_sample = 2000; % At 0.1 seconds
    phase_jump_angle = 30 * pi/180; % 30 degrees
    
    N = 8000;
    time = zeros(1, N);
    y9 = zeros(1, N);
    y10 = zeros(1, N);
    phase_error = zeros(1, N);
    
    figure('Position', [100, 100, 1400, 600]);
    
    for a = 1:N
        time(a) = a * Ts;
        
        % Grid with phase jump
        grid_wt_m = grid_wt_m - 0.015707963267949;
        
        if a == phase_jump_sample
            grid_wt_m = grid_wt_m + phase_jump_angle; % Sudden 30° jump
            fprintf('Phase jump applied at t=%.3f sec\n', time(a));
        end
        
        grid_wt_m = atan2(sin(grid_wt_m), cos(grid_wt_m));
        y9(a) = 1 * cos(grid_wt_m);
        
        % Inverter control (same as Test 1)
        if (a <= 400)
            theta_ref = (-2 * M_PI * 50 * t1) + pi/2;
            theta_ref = atan2(sin(theta_ref), cos(theta_ref));
            y10(a) = 1 * cos(theta_ref);
            wt_prev = WTinv_update;
            WTinv_update = theta_ref;
            delta_w_raw = WTinv_update - wt_prev;
            if delta_w_raw > pi
                delta_w = delta_w_raw - 2*pi;
            elseif delta_w_raw < -pi
                delta_w = delta_w_raw + 2*pi;
            else
                delta_w = delta_w_raw;
            end
            t1 = t1 + Ts;
        elseif (a > 400) && (a <= 800)
            WTinv_update = WTinv_update + delta_w;
            WTinv_update = atan2(sin(WTinv_update), cos(WTinv_update));
            y10(a) = 1 * cos(WTinv_update);
        elseif (a > 800) && (a <= 3000)
            WTinv_update = WTinv_update + delta_w;
            grid_wt_error = sin(grid_wt_m - WTinv_update);
            delta_w_sync = wKp * grid_wt_error + wKp1 * (grid_wt_error - wt_error_prev);
            wt_error_prev = grid_wt_error;
            WTinv_update = WTinv_update + delta_w_sync;
            WTinv_update = atan2(sin(WTinv_update), cos(WTinv_update));
            y10(a) = 1 * cos(WTinv_update);
            phase_error(a) = grid_wt_error * 180/pi; % Convert to degrees
        elseif (a > 3000)
            WTinv_update = WTinv_update + delta_w;
            WTinv_update = atan2(sin(WTinv_update), cos(WTinv_update));
            y10(a) = 1 * cos(WTinv_update);
        end
    end
    
    subplot(2,1,1);
    plot(time*1000, y9, 'r-', 'LineWidth', 1.5); hold on;
    plot(time*1000, y10, 'b--', 'LineWidth', 1.5);
    xlabel('Time (ms)'); ylabel('Voltage (pu)');
    title('30° Phase Jump Response');
    legend('Grid', 'Inverter');
    grid on;
    xlim([90 150]);
    xline(phase_jump_sample*Ts*1000, 'k--', 'Phase Jump');
    
    subplot(2,1,2);
    plot(time*1000, phase_error, 'g-', 'LineWidth', 1.5);
    xlabel('Time (ms)'); ylabel('Phase Error (degrees)');
    title('Phase Error During Synchronization');
    grid on;
    xlim([90 150]);
    xline(phase_jump_sample*Ts*1000, 'k--', 'Phase Jump');
end

%% TEST CASE 3: Frequency Jump Test
function run_frequency_jump_test()
    Ts = 5e-5;
    M_PI = pi;
    t1 = 0;
    WTinv_update = pi/2;
    
    wKp = 0.001;
    wKp1 = 0.001;
    
    grid_wt_m = pi/2;
    grid_wt_error = 0;
    delta_w_sync = 0;
    wt_error_prev = 0;
    delta_w = 0;
    
    freq_jump_sample = 2000;
    
    N = 8000;
    time = zeros(1, N);
    y9 = zeros(1, N);
    y10 = zeros(1, N);
    freq_array = zeros(1, N);
    
    figure('Position', [100, 100, 1400, 600]);
    
    for a = 1:N
        time(a) = a * Ts;
        
        % Grid with frequency jump
        if a < freq_jump_sample
            delta_w_grid = -2*pi*50*Ts; % 50Hz
        else
            delta_w_grid = -2*pi*51*Ts; % 51Hz
        end
        
        grid_wt_m = grid_wt_m + delta_w_grid;
        grid_wt_m = atan2(sin(grid_wt_m), cos(grid_wt_m));
        y9(a) = 1 * cos(grid_wt_m);
        
        % Inverter control
        if (a <= 400)
            theta_ref = (-2 * M_PI * 50 * t1) + pi/2;
            theta_ref = atan2(sin(theta_ref), cos(theta_ref));
            y10(a) = 1 * cos(theta_ref);
            wt_prev = WTinv_update;
            WTinv_update = theta_ref;
            delta_w_raw = WTinv_update - wt_prev;
            if delta_w_raw > pi
                delta_w = delta_w_raw - 2*pi;
            elseif delta_w_raw < -pi
                delta_w = delta_w_raw + 2*pi;
            else
                delta_w = delta_w_raw;
            end
            t1 = t1 + Ts;
        elseif (a > 400) && (a <= 800)
            WTinv_update = WTinv_update + delta_w;
            WTinv_update = atan2(sin(WTinv_update), cos(WTinv_update));
            y10(a) = 1 * cos(WTinv_update);
        elseif (a > 800) && (a <= 3000)
            WTinv_update = WTinv_update + delta_w;
            grid_wt_error = sin(grid_wt_m - WTinv_update);
            delta_w_sync = wKp * grid_wt_error + wKp1 * (grid_wt_error - wt_error_prev);
            wt_error_prev = grid_wt_error;
            WTinv_update = WTinv_update + delta_w_sync;
            WTinv_update = atan2(sin(WTinv_update), cos(WTinv_update));
            y10(a) = 1 * cos(WTinv_update);
        elseif (a > 3000)
            WTinv_update = WTinv_update + delta_w;
            WTinv_update = atan2(sin(WTinv_update), cos(WTinv_update));
            y10(a) = 1 * cos(WTinv_update);
        end
        
        freq_array(a) = delta_w / (2*pi*Ts);
    end
    
    subplot(2,1,1);
    plot(time*1000, y9, 'r-', 'LineWidth', 1.5); hold on;
    plot(time*1000, y10, 'b--', 'LineWidth', 1.5);
    xlabel('Time (ms)'); ylabel('Voltage (pu)');
    title('Frequency Jump Response (50Hz to 51Hz)');
    legend('Grid', 'Inverter');
    grid on;
    xlim([90 150]);
    
    subplot(2,1,2);
    plot(time*1000, abs(freq_array), 'm-', 'LineWidth', 1.5);
    xlabel('Time (ms)'); ylabel('Frequency (Hz)');
    title('Inverter Frequency Tracking');
    grid on;
    xlim([90 150]);
    yline(50, 'r--', '50Hz');
    yline(51, 'g--', '51Hz');
end

%% TEST CASE 4: Comparison Test (WITH vs WITHOUT unwrapping)
function run_comparison_test()
    Ts = 5e-5;
    M_PI = pi;
    N = 1000;
    
    % WITH unwrapping
    theta_unwrapped = zeros(1, N);
    delta_w_unwrapped = zeros(1, N);
    t1 = 0;
    WTinv_update = pi/2;
    
    for a = 1:N
        theta_ref = (-2 * M_PI * 50 * t1) + pi/2;
        theta_ref = atan2(sin(theta_ref), cos(theta_ref)); % Unwrap
        
        wt_prev = WTinv_update;
        WTinv_update = theta_ref;
        delta_w_raw = WTinv_update - wt_prev;
        
        if delta_w_raw > pi
            delta_w = delta_w_raw - 2*pi;
        elseif delta_w_raw < -pi
            delta_w = delta_w_raw + 2*pi;
        else
            delta_w = delta_w_raw;
        end
        
        theta_unwrapped(a) = theta_ref;
        delta_w_unwrapped(a) = delta_w;
        t1 = t1 + Ts;
    end
    
    % WITHOUT unwrapping 
    theta_buggy = zeros(1, N);
    delta_w_buggy = zeros(1, N);
    t1 = 0;
    WTinv_update = 0;
    
    for a = 1:N
        theta_ref = (-2 * M_PI * 50 * t1) + pi/2;
        
        % Original reset
        if (theta_ref < -3*pi/2)
            theta_ref = theta_ref - 2*pi;
            t1 = 0;
        end
        
        wt_prev = WTinv_update;
        WTinv_update = theta_ref;
        delta_w = WTinv_update - wt_prev;
        
        theta_buggy(a) = theta_ref;
        delta_w_buggy(a) = delta_w;
        t1 = t1 + Ts;
    end
    
    time = (1:N) * Ts * 1000;
    
    figure('Position', [100, 100, 1400, 800]);
    
    subplot(3,1,1);
    plot(time, theta_unwrapped, 'b-', 'LineWidth', 2); hold on;
    plot(time, theta_buggy, 'r--', 'LineWidth', 2);
    xlabel('Time (ms)'); ylabel('Angle (rad)');
    title('Phase Angle Comparison');
    legend('WITH Unwrapping', 'WITHOUT Unwrapping (Buggy)');
    grid on;
    
    subplot(3,1,2);
    plot(time, delta_w_unwrapped, 'b-', 'LineWidth', 2); hold on;
    plot(time, delta_w_buggy, 'r--', 'LineWidth', 2);
    xlabel('Time (ms)'); ylabel('delta\_w (rad/sample)');
    title('Angular Velocity Comparison');
    legend('WITH Unwrapping', 'WITHOUT Unwrapping (Buggy)');
    grid on;
    yline(-0.015708, 'k--', 'Expected 50Hz');
    
    subplot(3,1,3);
    freq_unwrapped = delta_w_unwrapped / (2*pi*Ts);
    freq_buggy = delta_w_buggy / (2*pi*Ts);
    plot(time, abs(freq_unwrapped), 'b-', 'LineWidth', 2); hold on;
    plot(time, abs(freq_buggy), 'r--', 'LineWidth', 2);
    xlabel('Time (ms)'); ylabel('Frequency (Hz)');
    title('Frequency Comparison');
    legend('WITH Unwrapping', 'WITHOUT Unwrapping (Buggy)');
    grid on;
    yline(50, 'k--', '50Hz reference');
    ylim([0 500]);
    
    fprintf('\n=== COMPARISON RESULTS ===\n');
    fprintf('WITH Unwrapping:\n');
    fprintf('  Average delta_w: %.6f rad/sample\n', mean(delta_w_unwrapped(100:end)));
    fprintf('  Average frequency: %.2f Hz\n', mean(abs(freq_unwrapped(100:end))));
    fprintf('  Std dev delta_w: %.6f\n', std(delta_w_unwrapped(100:end)));
    fprintf('\nWITHOUT Unwrapping (Buggy):\n');
    fprintf('  Average delta_w: %.6f rad/sample\n', mean(delta_w_buggy(100:end)));
    fprintf('  Average frequency: %.2f Hz\n', mean(abs(freq_buggy(100:end))));
    fprintf('  Std dev delta_w: %.6f\n', std(delta_w_buggy(100:end)));
end