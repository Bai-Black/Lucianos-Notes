%% 情景三：平衡pitch环 - PI控制器仿真
% 作者：Luciano
% 日期：2026-06-09
% 描述：二阶振荡系统 G4(s) = 1/(0.04s^2 + 0.1s + 1) * e^(-Ls)
%       T_eq = 0.05
%       输出文件夹：3_1

function results = run_scenario3_PI()
    clc; clear; close all;

    %% ========== 系统参数 ==========
    K = 1;
    a2 = 0.04;       % s^2系数
    a1 = 0.1;        % s系数
    a0 = 1;          % 常数项
    T_eq = 0.05;
    u_lim = 1;

    % 计算固有频率和阻尼比
    omega_n = sqrt(a0 / a2);       % 5 rad/s
    zeta = a1 / (2 * sqrt(a0 * a2));  % 0.25 (欠阻尼)

    t_sim = 20;
    Ts = 0.001;
    N = t_sim / Ts;
    t = (0:N-1) * Ts;

    L_T_values = [0.02, 0.1, 0.3, 0.5];
    x_values = [0.2, 0.5, 0.8];

    w_low = linspace(0.05, 0.5, 30);
    w_high = linspace(0.5, 3, 30);
    w_values = [w_low, w_high(2:end)];

    strategies = {'DBC_CLA', 'IBC', 'CI', 'CBC1', 'CBC2', 'DBC_STR'};
    num_strategies = length(strategies);

    output_folder = '3_1';
    if ~exist(output_folder, 'dir'), mkdir(output_folder); end

    %% 初始化
    num_L_T = length(L_T_values);
    num_x = length(x_values);
    num_w = length(w_values);

    R_S_matrix = zeros(num_L_T, num_x, num_w);
    IAE_matrix = zeros(num_L_T, num_x, num_w, num_strategies);
    ITAE_matrix = zeros(num_L_T, num_x, num_w, num_strategies);

    %% ========== 第一阶段：计算R_S ==========
    fprintf('=== 情景三 PI控制器仿真 ===\n');
    fprintf('系统: G4(s) = 1/(0.04s^2 + 0.1s + 1) * e^(-Ls)\n');
    fprintf('omega_n = %.2f rad/s, zeta = %.3f (欠阻尼)\n', omega_n, zeta);
    fprintf('==========================================\n');

    for i = 1:num_L_T
        L_T = L_T_values(i);
        L = L_T * T_eq;
        for j = 1:num_x
            x = x_values(j);
            lambda = x * T_eq;

            Kp = T_eq / (K * (lambda + L));
            Ti = T_eq;
            Ki = Kp / Ti;

            fprintf('[R_S] L/T=%.2f, x=%.1f ...', L_T, x);
            for w_idx = 1:num_w
                w_val = w_values(w_idx);
                w_signal = zeros(N, 1);
                w_signal(round(1/Ts):end) = w_val;

                [~, u_f_max] = simulate_without_saturation_osc(K, a2, a1, a0, L, Kp, Ki, Ts, N, w_signal);
                u_0 = 0;
                if u_f_max > u_lim
                    R_S_matrix(i, j, w_idx) = (u_f_max - u_lim) / (u_f_max - u_0);
                else
                    R_S_matrix(i, j, w_idx) = 0;
                end
            end
            fprintf(' 完成\n');
        end
    end

    %% ========== 第二阶段：计算IAE/ITAE ==========
    fprintf('\n阶段2: 计算IAE/ITAE...\n');

    total = num_L_T * num_x;
    count = 0;
    discarded = 0;

    for i = 1:num_L_T
        L_T = L_T_values(i);
        L = L_T * T_eq;
        for j = 1:num_x
            x = x_values(j);
            lambda = x * T_eq;

            count = count + 1;
            fprintf('[%d/%d] L/T=%.2f, x=%.1f ...', count, total, L_T, x);

            Kp = T_eq / (K * (lambda + L));
            Ti = T_eq;
            Ki = Kp / Ti;

            for w_idx = 1:num_w
                w_val = w_values(w_idx);
                w_signal = zeros(N, 1);
                w_signal(round(1/Ts):end) = w_val;

                for s = 1:num_strategies
                    [IAE_val, ITAE_val, ~, ~, converged] = simulate_pi_osc(...
                        K, a2, a1, a0, L, Kp, Ki, Ts, N, w_signal, u_lim, strategies{s});
                    if converged
                        IAE_matrix(i, j, w_idx, s) = IAE_val;
                        ITAE_matrix(i, j, w_idx, s) = ITAE_val;
                    else
                        IAE_matrix(i, j, w_idx, s) = NaN;
                        ITAE_matrix(i, j, w_idx, s) = NaN;
                        discarded = discarded + 1;
                    end
                end
            end
            fprintf(' 完成\n');
        end
    end

    fprintf('丢弃数据点: %d\n', discarded);

    results.L_T_values = L_T_values;
    results.x_values = x_values;
    results.w_values = w_values;
    results.strategies = strategies;
    results.R_S = R_S_matrix;
    results.IAE = IAE_matrix;
    results.ITAE = ITAE_matrix;

    %% ========== 第三阶段：绘图 ==========
    fprintf('\n绘图...\n');
    plot_normalized_results(results, output_folder);
    plot_time_domain(results, K, a2, a1, a0, T_eq, Ts, N, w_values, u_lim, output_folder);

    save('scenario3_PI_results.mat', 'results');
    fprintf('完成！结果保存到 scenario3_PI_results.mat\n');
end

%% ========== 无饱和仿真（振荡二阶系统） ==========
function [y, u_max] = simulate_without_saturation_osc(K, a2, a1, a0, L, Kp, Ki, Ts, N, w)
    y = zeros(N, 1);
    u_c = zeros(N, 1);
    e = zeros(N, 1);
    u_i = zeros(N, 1);
    % 状态变量：y, dy/dt
    y1 = zeros(N, 1);  % y
    y2 = zeros(N, 1);  % dy/dt
    delay_steps = round(L / Ts);

    for k = 2:N
        e(k) = w(k) - y(k-1);
        u_p = Kp * e(k);
        u_i(k) = u_i(k-1) + Ki * Ts * e(k);
        u_c(k) = u_p + u_i(k);

        if k > delay_steps
            u_delayed = u_c(k - delay_steps);
        else
            u_delayed = 0;
        end

        % 二阶振荡系统：a2*y'' + a1*y' + a0*y = K*u
        % 状态空间：y1' = y2, y2' = (K*u - a1*y2 - a0*y1) / a2
        dy1 = y2(k-1);
        dy2 = (K * u_delayed - a1 * y2(k-1) - a0 * y1(k-1)) / a2;

        y1(k) = y1(k-1) + Ts * dy1;
        y2(k) = y2(k-1) + Ts * dy2;
        y(k) = y1(k);
    end
    u_max = max(abs(u_c));
end

%% ========== PI仿真（振荡二阶系统） ==========
function [IAE, ITAE, y, u_sat, converged] = simulate_pi_osc(K, a2, a1, a0, L, Kp, Ki, Ts, N, w, u_lim, strategy)
    y = zeros(N, 1);
    u_c = zeros(N, 1);
    u_sat = zeros(N, 1);
    e = zeros(N, 1);
    e_sat = zeros(N, 1);
    u_i = zeros(N, 1);
    y1 = zeros(N, 1);
    y2 = zeros(N, 1);

    delay_steps = round(L / Ts);
    y_prev = 0;
    y_prev2 = 0;

    for k = 2:N
        e(k) = w(k) - y(k-1);

        params.e = e(k);
        params.e_prev = e(max(k-1, 1));
        params.e_sat_prev = e_sat(k-1);
        params.Kp = Kp;
        params.Ki = Ki;
        params.Ts = Ts;
        params.u_i_prev = u_i(k-1);
        params.u_c_prev = u_c(max(k-1, 1));
        params.u_sat_prev = u_sat(max(k-1, 1));
        params.u_lim = u_lim;
        params.strategy = strategy;
        params.y = y(max(k-1, 1));
        params.y_prev = y_prev;
        params.y_prev2 = y_prev2;
        params.w = w(k);
        params.K = K;
        params.T = a2/a1;  % 等效时间常数
        params.L = L;

        [u_c(k), u_sat(k), u_i(k), e_sat(k)] = PI(params);

        if k > delay_steps
            u_delayed = u_sat(k - delay_steps);
        else
            u_delayed = 0;
        end

        % 二阶振荡系统
        dy1 = y2(k-1);
        dy2 = (K * u_delayed - a1 * y2(k-1) - a0 * y1(k-1)) / a2;
        y1(k) = y1(k-1) + Ts * dy1;
        y2(k) = y2(k-1) + Ts * dy2;
        y(k) = y1(k);

        y_prev2 = y_prev;
        y_prev = y(k-1);
    end

    final_error = abs(y(end) - w(end)) / max(abs(w(end)), 0.01);
    converged = final_error < 0.05;

    IAE = sum(abs(e)) * Ts;
    ITAE = sum((1:N)' .* abs(e)) * Ts;
end

%% ========== 归一化图表 ==========
function plot_normalized_results(results, output_folder)
    num_L_T = length(results.L_T_values);
    num_x = length(results.x_values);
    num_strategies = length(results.strategies);
    strategies = results.strategies;
    idx_dbc = find(strcmp(strategies, 'DBC_CLA'));
    colors = lines(num_strategies);

    for i = 1:num_L_T
        for j = 1:num_x
            R_S_vec = squeeze(results.R_S(i, j, :));

            figure('Visible', 'off');
            hold on;
            has_data = false;
            for s = 1:num_strategies
                IAE_vec = squeeze(results.IAE(i, j, :, s));
                valid = R_S_vec >= 0.1 & R_S_vec <= 0.95 & ~isnan(IAE_vec) & ~isnan(squeeze(results.IAE(i, j, :, idx_dbc)));
                if sum(valid) < 2, continue; end
                R_S_valid = R_S_vec(valid);
                IAE_base = squeeze(results.IAE(i, j, valid, idx_dbc));
                plot(R_S_valid, IAE_vec(valid) ./ IAE_base, '-o', 'Color', colors(s,:), 'LineWidth', 2, 'MarkerSize', 4, 'DisplayName', strategies{s});
                has_data = true;
            end
            if has_data
                xlabel('R_S'); ylabel('归一化 IAE');
                title(sprintf('IAE归一化 (L/T=%.2f, x=%.1f)', results.L_T_values(i), results.x_values(j)));
                legend('Location', 'best'); grid on; xlim([0.1, 0.95]); ylim([0.5, 2.5]);
            end
            hold off;
            if has_data, saveas(gcf, fullfile(output_folder, sprintf('IAE_norm_LT%.2f_x%.1f.png', results.L_T_values(i), results.x_values(j)))); end

            figure('Visible', 'off');
            hold on;
            has_data = false;
            for s = 1:num_strategies
                ITAE_vec = squeeze(results.ITAE(i, j, :, s));
                valid = R_S_vec >= 0.1 & R_S_vec <= 0.95 & ~isnan(ITAE_vec) & ~isnan(squeeze(results.ITAE(i, j, :, idx_dbc)));
                if sum(valid) < 2, continue; end
                R_S_valid = R_S_vec(valid);
                ITAE_base = squeeze(results.ITAE(i, j, valid, idx_dbc));
                plot(R_S_valid, ITAE_vec(valid) ./ ITAE_base, '-o', 'Color', colors(s,:), 'LineWidth', 2, 'MarkerSize', 4, 'DisplayName', strategies{s});
                has_data = true;
            end
            if has_data
                xlabel('R_S'); ylabel('归一化 ITAE');
                title(sprintf('ITAE归一化 (L/T=%.2f, x=%.1f)', results.L_T_values(i), results.x_values(j)));
                legend('Location', 'best'); grid on; xlim([0.1, 0.95]); ylim([0.5, 2.5]);
            end
            hold off;
            if has_data, saveas(gcf, fullfile(output_folder, sprintf('ITAE_norm_LT%.2f_x%.1f.png', results.L_T_values(i), results.x_values(j)))); end
        end
    end
    close all;
end

%% ========== 时域图 ==========
function plot_time_domain(results, K, a2, a1, a0, T_eq, Ts, N, w_values, u_lim, output_folder)
    num_L_T = length(results.L_T_values);
    num_x = length(results.x_values);
    num_strategies = length(results.strategies);
    strategies = results.strategies;
    t = (0:N-1) * Ts;
    colors = lines(num_strategies);

    fprintf('绘制时域图...\n');

    for i = 1:num_L_T
        L_T = results.L_T_values(i);
        L = L_T * T_eq;
        if L_T <= 0.1, target_RS = 0.2; else, target_RS = 0.3; end

        for j = 1:num_x
            x = results.x_values(j);
            lambda = x * T_eq;

            R_S_vec = squeeze(results.R_S(i, j, :));
            [~, sort_idx] = sort(abs(R_S_vec - target_RS));

            w_val = NaN;
            for idx = sort_idx'
                all_ok = true;
                for s = 1:num_strategies
                    if isnan(results.IAE(i, j, idx, s)), all_ok = false; break; end
                end
                if all_ok, w_val = w_values(idx); break; end
            end

            if isnan(w_val), fprintf('  L/T=%.2f, x=%.1f: 跳过\n', L_T, x); continue; end
            fprintf('  L/T=%.2f, x=%.1f: w=%.3f, R_S≈%.1f\n', L_T, x, w_val, target_RS);

            w_signal = zeros(N, 1);
            w_signal(round(1/Ts):end) = w_val;

            Kp = T_eq / (K * (lambda + L));
            Ti = T_eq;
            Ki = Kp / Ti;

            y_all = zeros(N, num_strategies);
            u_sat_all = zeros(N, num_strategies);
            for s = 1:num_strategies
                [~, ~, y_all(:, s), u_sat_all(:, s)] = simulate_pi_osc(...
                    K, a2, a1, a0, L, Kp, Ki, Ts, N, w_signal, u_lim, strategies{s});
            end

            show_idx = t <= 5;

            figure('Visible', 'off');
            hold on;
            plot(t(show_idx), w_signal(show_idx), 'k--', 'LineWidth', 2, 'DisplayName', 'w');
            for s = 1:num_strategies
                plot(t(show_idx), y_all(show_idx, s), '-', 'Color', colors(s,:), 'LineWidth', 1.5, 'DisplayName', strategies{s});
            end
            xlabel('时间 (s)'); ylabel('输出 y');
            title(sprintf('系统输出 (L/T=%.2f, x=%.1f, R_S≈%.1f)', L_T, x, target_RS));
            legend('Location', 'best', 'FontSize', 9); grid on; hold off;
            saveas(gcf, fullfile(output_folder, sprintf('y_RS%.0f_LT%.2f_x%.1f.png', target_RS*10, L_T, x)));

            figure('Visible', 'off');
            hold on;
            yline(u_lim, 'k--', 'LineWidth', 1.5, 'DisplayName', 'u_{lim}');
            yline(-u_lim, 'k--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
            for s = 1:num_strategies
                plot(t(show_idx), u_sat_all(show_idx, s), '-', 'Color', colors(s,:), 'LineWidth', 1.5, 'DisplayName', strategies{s});
            end
            xlabel('时间 (s)'); ylabel('u_{sat}');
            title(sprintf('控制器输出 (L/T=%.2f, x=%.1f, R_S≈%.1f)', L_T, x, target_RS));
            legend('Location', 'best', 'FontSize', 9); grid on; ylim([-1.5, 1.5]); hold off;
            saveas(gcf, fullfile(output_folder, sprintf('u_sat_RS%.0f_LT%.2f_x%.1f.png', target_RS*10, L_T, x)));
        end
    end
    close all;
end
