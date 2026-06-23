%% 情景一：简化速度环 - PID控制器仿真
% 作者：Luciano
% 日期：2026-06-09
% 描述：使用PID控制器遍历L/T、x、w的组合
%       以DBC_CLA为基准归一化IAE和ITAE
%       L/T≤0.1时绘制R_S≈0.2的时域图，其他绘制R_S≈0.3的时域图

function results = run_scenario1_PID()
    %% 清理工作区
    clc; clear; close all;

    %% ========== 1. 输入参数 ==========
    % 系统参数
    K = 1;           % 系统增益
    T = 0.03;        % 时间常数
    u_lim = 1;       % 饱和限制（±1）

    % 仿真参数
    t_sim = 20;      % 仿真时长（秒）
    Ts = 0.001;      % 采样时间
    N = t_sim / Ts;  % 仿真步数
    t = (0:N-1) * Ts;  % 时间向量

    % 参数范围
    L_T_values = [0.02, 0.1, 0.3, 0.5];
    x_values = [0.2, 0.5, 0.8];

    % 设定值范围
    w_low = linspace(0.05, 0.5, 30);
    w_high = linspace(0.5, 3, 30);
    w_values = [w_low, w_high(2:end)];

    % 抗积分饱和策略列表（包含GBC）
    strategies = {'DBC_CLA', 'IBC', 'CI', 'CBC1', 'CBC2', 'DBC_STR', 'GBC'};
    num_strategies = length(strategies);

    % 创建输出文件夹
    output_folder = '1_2';
    if ~exist(output_folder, 'dir')
        mkdir(output_folder);
    end

    %% 初始化
    num_L_T = length(L_T_values);
    num_x = length(x_values);
    num_w = length(w_values);

    R_S_matrix = zeros(num_L_T, num_x, num_w);
    IAE_matrix = zeros(num_L_T, num_x, num_w, num_strategies);
    ITAE_matrix = zeros(num_L_T, num_x, num_w, num_strategies);

    %% ========== 第一阶段：计算R_S（无饱和仿真） ==========
    fprintf('阶段1: 计算R_S（无饱和仿真）...\n');
    fprintf('系统参数: K=%.1f, T=%.3f, u_lim=%.1f\n', K, T, u_lim);
    fprintf('==========================================\n');

    for i = 1:num_L_T
        L_T = L_T_values(i);
        L = L_T * T;

        for j = 1:num_x
            x = x_values(j);
            lambda = x * T;

            % λ方法整定PID参数
            Kp = (T + 0.5*L) / (K * (lambda + 0.5*L));
            Ti = T + 0.5*L;
            Ki = Kp / Ti;
            Td = T*L / (2*T + L);
            Kd = Kp * Td;

            fprintf('[R_S] L/T=%.2f, x=%.1f ...', L_T, x);

            for w_idx = 1:num_w
                w_val = w_values(w_idx);
                w_signal = zeros(N, 1);
                w_signal(round(1/Ts):end) = w_val;
                d = zeros(N, 1);

                [~, u_f_max] = simulate_without_saturation(K, T, L, Kp, Ki, Ts, N, w_signal, d);

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

    %% ========== 第二阶段：计算IAE/ITAE（有饱和仿真） ==========
    fprintf('\n==========================================\n');
    fprintf('阶段2: 计算IAE/ITAE（PID控制器）...\n');
    fprintf('策略: '); fprintf('%s ', strategies{:}); fprintf('\n');
    fprintf('==========================================\n');

    total = num_L_T * num_x;
    count = 0;
    discarded = 0;

    for i = 1:num_L_T
        L_T = L_T_values(i);
        L = L_T * T;

        for j = 1:num_x
            x = x_values(j);
            lambda = x * T;

            count = count + 1;
            fprintf('[%d/%d] L/T=%.2f, x=%.1f ...', count, total, L_T, x);

            % λ方法整定PID参数
            Kp = (T + 0.5*L) / (K * (lambda + 0.5*L));
            Ti = T + 0.5*L;
            Ki = Kp / Ti;
            Td = T*L / (2*T + L);
            Kd = Kp * Td;

            for w_idx = 1:num_w
                w_val = w_values(w_idx);
                w_signal = zeros(N, 1);
                w_signal(round(1/Ts):end) = w_val;
                d = zeros(N, 1);

                for s = 1:num_strategies
                    [IAE_val, ITAE_val, ~, ~, converged] = simulate_pid_controller(...
                        K, T, L, Kp, Ki, Kd, Ts, N, w_signal, d, u_lim, strategies{s});

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

    %% 打包结果
    results.L_T_values = L_T_values;
    results.x_values = x_values;
    results.w_values = w_values;
    results.strategies = strategies;
    results.R_S = R_S_matrix;
    results.IAE = IAE_matrix;
    results.ITAE = ITAE_matrix;

    %% ========== 第三阶段：绘图 ==========
    fprintf('\n==========================================\n');
    fprintf('仿真完成！开始绘图...\n');

    plot_normalized_results(results, output_folder);
    plot_time_domain(results, K, T, Ts, N, w_values, u_lim, output_folder);

    save('scenario1_PID_results.mat', 'results');
    fprintf('完成！结果保存到 scenario1_PID_results.mat\n');

end

%% ========== 无饱和仿真函数 ==========
function [y, u_max] = simulate_without_saturation(K, T, L, Kp, Ki, Ts, N, w, d)
    y = zeros(N, 1);
    u_c = zeros(N, 1);
    e = zeros(N, 1);
    u_i = zeros(N, 1);
    y_sys = zeros(N, 1);
    delay_steps = round(L / Ts);

    for k = 2:N
        e(k) = w(k) - y(k-1);
        u_p = Kp * e(k);
        u_i(k) = u_i(k-1) + Ki * Ts * e(k);
        u_c(k) = u_p + u_i(k);

        if k > delay_steps
            u_delayed = u_c(k - delay_steps) + d(k);
        else
            u_delayed = d(k);
        end
        y_sys(k) = y_sys(k-1) + Ts * (K * u_delayed - y_sys(k-1)) / T;
        y(k) = y_sys(k);
    end
    u_max = max(abs(u_c));
end

%% ========== PID控制器仿真函数 ==========
function [IAE, ITAE, y, u_sat, converged] = simulate_pid_controller(K, T, L, Kp, Ki, Kd, Ts, N, w, d, u_lim, strategy)
    y = zeros(N, 1);
    u_c = zeros(N, 1);
    u_sat = zeros(N, 1);
    e = zeros(N, 1);
    e_sat = zeros(N, 1);
    u_i = zeros(N, 1);
    u_d = zeros(N, 1);
    y_sys = zeros(N, 1);

    delay_steps = round(L / Ts);

    % 初始化历史变量
    e_prev = 0;
    e_prev2 = 0;
    d_prev = 0;
    y_prev = 0;
    y_prev2 = 0;
    u_d_prev = 0;

    for k = 2:N
        e(k) = w(k) - y(k-1);

        % 构建参数结构体
        params.e = e(k);
        params.e_prev = e_prev;
        params.e_prev2 = e_prev2;
        params.e_sat_prev = e_sat(k-1);
        params.Kp = Kp;
        params.Ki = Ki;
        params.Kd = Kd;
        params.Ts = Ts;
        params.u_i_prev = u_i(k-1);
        params.u_c_prev = u_c(max(k-1, 1));
        params.u_sat_prev = u_sat(max(k-1, 1));
        params.u_lim = u_lim;
        params.strategy = strategy;
        params.N_filter = 10;
        params.d_prev = d_prev;
        params.y = y(max(k-1, 1));
        params.y_prev = y_prev;
        params.y_prev2 = y_prev2;
        params.w = w(k);
        params.K = K;
        params.T = T;
        params.L = L;
        params.u_d_prev = u_d_prev;

        % 调用PID控制器
        [u_c(k), u_sat(k), u_i(k), e_sat(k), u_d(k)] = PID(params);

        % FOPDT系统响应
        if k > delay_steps
            u_delayed = u_sat(k - delay_steps) + d(k);
        else
            u_delayed = d(k);
        end
        y_sys(k) = y_sys(k-1) + Ts * (K * u_delayed - y_sys(k-1)) / T;
        y(k) = y_sys(k);

        % 更新历史变量
        e_prev2 = e_prev;
        e_prev = e(k);
        d_prev = u_d(k);
        y_prev2 = y_prev;
        y_prev = y(k-1);
        u_d_prev = u_d(k);
    end

    % 检查收敛（最终误差 < 5%）
    final_error = abs(y(end) - w(end)) / max(abs(w(end)), 0.01);
    converged = final_error < 0.05;

    IAE = sum(abs(e)) * Ts;
    ITAE = sum((1:N)' .* abs(e)) * Ts;
end

%% ========== 归一化图表绘图函数 ==========
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

            % IAE归一化图
            figure('Name', sprintf('IAE L/T=%.2f x=%.1f', ...
                results.L_T_values(i), results.x_values(j)), ...
                'Position', [100, 100, 800, 600], 'Visible', 'off');

            hold on;
            has_data = false;
            for s = 1:num_strategies
                IAE_vec = squeeze(results.IAE(i, j, :, s));
                valid = R_S_vec >= 0.1 & R_S_vec <= 0.95 & ~isnan(IAE_vec) & ~isnan(squeeze(results.IAE(i, j, :, idx_dbc)));

                if sum(valid) < 2
                    continue;
                end

                R_S_valid = R_S_vec(valid);
                IAE_base = squeeze(results.IAE(i, j, valid, idx_dbc));
                IAE_norm = IAE_vec(valid) ./ IAE_base;

                plot(R_S_valid, IAE_norm, '-o', 'Color', colors(s,:), ...
                    'LineWidth', 2, 'MarkerSize', 4, 'DisplayName', strategies{s});
                has_data = true;
            end

            if has_data
                xlabel('R_S', 'FontSize', 12);
                ylabel('归一化 IAE (基准: DBC\_CLA)', 'FontSize', 12);
                title(sprintf('IAE归一化 (L/T=%.2f, x=%.1f)', ...
                    results.L_T_values(i), results.x_values(j)), 'FontSize', 14);
                legend('Location', 'best', 'FontSize', 10);
                grid on; xlim([0.1, 0.95]); ylim([0.5, 2.5]);
            end
            hold off;

            if has_data
                saveas(gcf, fullfile(output_folder, ...
                    sprintf('IAE_norm_LT%.2f_x%.1f.png', ...
                    results.L_T_values(i), results.x_values(j))));
            end

            % ITAE归一化图
            figure('Name', sprintf('ITAE L/T=%.2f x=%.1f', ...
                results.L_T_values(i), results.x_values(j)), ...
                'Position', [100, 100, 800, 600], 'Visible', 'off');

            hold on;
            has_data = false;
            for s = 1:num_strategies
                ITAE_vec = squeeze(results.ITAE(i, j, :, s));
                valid = R_S_vec >= 0.1 & R_S_vec <= 0.95 & ~isnan(ITAE_vec) & ~isnan(squeeze(results.ITAE(i, j, :, idx_dbc)));

                if sum(valid) < 2
                    continue;
                end

                R_S_valid = R_S_vec(valid);
                ITAE_base = squeeze(results.ITAE(i, j, valid, idx_dbc));
                ITAE_norm = ITAE_vec(valid) ./ ITAE_base;

                plot(R_S_valid, ITAE_norm, '-o', 'Color', colors(s,:), ...
                    'LineWidth', 2, 'MarkerSize', 4, 'DisplayName', strategies{s});
                has_data = true;
            end

            if has_data
                xlabel('R_S', 'FontSize', 12);
                ylabel('归一化 ITAE (基准: DBC\_CLA)', 'FontSize', 12);
                title(sprintf('ITAE归一化 (L/T=%.2f, x=%.1f)', ...
                    results.L_T_values(i), results.x_values(j)), 'FontSize', 14);
                legend('Location', 'best', 'FontSize', 10);
                grid on; xlim([0.1, 0.95]); ylim([0.5, 2.5]);
            end
            hold off;

            if has_data
                saveas(gcf, fullfile(output_folder, ...
                    sprintf('ITAE_norm_LT%.2f_x%.1f.png', ...
                    results.L_T_values(i), results.x_values(j))));
            end
        end
    end
    close all;
end

%% ========== 时域图绘图函数 ==========
function plot_time_domain(results, K_sys, T_sys, Ts, N, w_values, u_lim, output_folder)
    num_L_T = length(results.L_T_values);
    num_x = length(results.x_values);
    num_strategies = length(results.strategies);
    strategies = results.strategies;
    t = (0:N-1) * Ts;

    colors = lines(num_strategies);

    fprintf('绘制时域图 (L/T≤0.1: R_S≈0.2, 其他: R_S≈0.3)...\n');

    for i = 1:num_L_T
        L_T = results.L_T_values(i);
        L = L_T * T_sys;

        if L_T <= 0.1
            target_RS = 0.2;
        else
            target_RS = 0.3;
        end

        for j = 1:num_x
            x = results.x_values(j);
            lambda = x * T_sys;

            % 找到R_S最接近目标值且所有策略都收敛的w值
            R_S_vec = squeeze(results.R_S(i, j, :));
            [~, sort_idx] = sort(abs(R_S_vec - target_RS));

            w_val = NaN;
            for idx = sort_idx'
                all_converged = true;
                for s = 1:num_strategies
                    if isnan(results.IAE(i, j, idx, s))
                        all_converged = false;
                        break;
                    end
                end
                if all_converged
                    w_val = w_values(idx);
                    break;
                end
            end

            if isnan(w_val)
                fprintf('  L/T=%.2f, x=%.1f: 未找到满足条件的w值，跳过\n', L_T, x);
                continue;
            end

            [~, best_idx] = min(abs(w_values - w_val));
            fprintf('  L/T=%.2f, x=%.1f: w=%.3f, R_S=%.3f (目标%.1f)\n', L_T, x, w_val, R_S_vec(best_idx), target_RS);

            % 运行仿真获取时域数据
            w_signal = zeros(N, 1);
            w_signal(round(1/Ts):end) = w_val;
            d = zeros(N, 1);

            Kp = (T_sys + 0.5*L) / (K_sys * (lambda + 0.5*L));
            Ti = T_sys + 0.5*L;
            Ki = Kp / Ti;
            Td = T_sys*L / (2*T_sys + L);
            Kd = Kp * Td;

            y_all = zeros(N, num_strategies);
            u_sat_all = zeros(N, num_strategies);

            for s = 1:num_strategies
                [~, ~, y_all(:, s), u_sat_all(:, s)] = simulate_pid_controller(...
                    K_sys, T_sys, L, Kp, Ki, Kd, Ts, N, w_signal, d, u_lim, strategies{s});
            end

            show_idx = t <= 5;

            % 输出-时间图
            figure('Name', sprintf('y L/T=%.2f x=%.1f', L_T, x), ...
                'Position', [100, 100, 900, 500], 'Visible', 'off');

            hold on;
            plot(t(show_idx), w_signal(show_idx), 'k--', 'LineWidth', 2, 'DisplayName', 'w');
            for s = 1:num_strategies
                plot(t(show_idx), y_all(show_idx, s), '-', 'Color', colors(s,:), ...
                    'LineWidth', 1.5, 'DisplayName', strategies{s});
            end
            xlabel('时间 (s)', 'FontSize', 12);
            ylabel('输出 y', 'FontSize', 12);
            title(sprintf('系统输出 (L/T=%.2f, x=%.1f, R_S≈%.1f)', L_T, x, target_RS), 'FontSize', 14);
            legend('Location', 'best', 'FontSize', 9);
            grid on; hold off;

            saveas(gcf, fullfile(output_folder, sprintf('y_RS%.0f_LT%.2f_x%.1f.png', target_RS*10, L_T, x)));

            % 控制器行为-时间图
            figure('Name', sprintf('u L/T=%.2f x=%.1f', L_T, x), ...
                'Position', [100, 100, 900, 500], 'Visible', 'off');

            hold on;
            yline(u_lim, 'k--', 'LineWidth', 1.5, 'DisplayName', 'u_{lim}');
            yline(-u_lim, 'k--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
            for s = 1:num_strategies
                plot(t(show_idx), u_sat_all(show_idx, s), '-', 'Color', colors(s,:), ...
                    'LineWidth', 1.5, 'DisplayName', strategies{s});
            end
            xlabel('时间 (s)', 'FontSize', 12);
            ylabel('u_{sat}', 'FontSize', 12);
            title(sprintf('控制器输出 (L/T=%.2f, x=%.1f, R_S≈%.1f)', L_T, x, target_RS), 'FontSize', 14);
            legend('Location', 'best', 'FontSize', 9);
            grid on; ylim([-1.5, 1.5]); hold off;

            saveas(gcf, fullfile(output_folder, sprintf('u_sat_RS%.0f_LT%.2f_x%.1f.png', target_RS*10, L_T, x)));
        end
    end
    close all;
end
