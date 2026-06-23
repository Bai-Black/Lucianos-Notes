%% 情景一：简化速度环 - 曲线跟踪仿真
% 作者：Luciano
% 日期：2026-06-09
% 描述：使用平滑曲线作为设定值，比较PI/PID/PIDF控制器
%       输出：1_3(PI), 1_4(PID), 1_5(PIDF)

function results = run_scenario1_curve()
    %% 清理工作区
    clc; clear; close all;

    %% ========== 系统参数 ==========
    K = 1;
    T = 0.03;
    u_lim = 1;

    t_sim = 30;
    Ts = 0.001;
    N = t_sim / Ts;
    t = (0:N-1) * Ts;

    L_T_values = [0.02, 0.1, 0.3, 0.5];
    x_values = [0.2, 0.5, 0.8];

    % 策略列表
    strategies_pi = {'DBC_CLA', 'IBC', 'CI', 'CBC1', 'CBC2', 'DBC_STR'};  % PI用6种
    strategies_pid = {'DBC_CLA', 'IBC', 'CI', 'CBC1', 'CBC2', 'DBC_STR', 'GBC'};  % PID/PIDF用7种
    num_strategies_pi = length(strategies_pi);
    num_strategies_pid = length(strategies_pid);

    % 创建输出文件夹
    folders = {'1_3', '1_4', '1_5'};
    for f = 1:length(folders)
        if ~exist(folders{f}, 'dir')
            mkdir(folders{f});
        end
    end

    %% ========== 生成平滑曲线设定值 ==========
    % 使用多个正弦波叠加生成复杂平滑曲线
    w = generate_smooth_curve(t, N);

    %% ========== 初始化 ==========
    num_L_T = length(L_T_values);
    num_x = length(x_values);

    results.L_T_values = L_T_values;
    results.x_values = x_values;
    results.strategies_pi = strategies_pi;
    results.strategies_pid = strategies_pid;
    results.t = t;
    results.w = w;

    % 存储各控制器结果
    results.PI.IAE = zeros(num_L_T, num_x, num_strategies_pi);
    results.PI.ITAE = zeros(num_L_T, num_x, num_strategies_pi);
    results.PID.IAE = zeros(num_L_T, num_x, num_strategies_pid);
    results.PID.ITAE = zeros(num_L_T, num_x, num_strategies_pid);
    results.PIDF.IAE = zeros(num_L_T, num_x, num_strategies_pid);
    results.PIDF.ITAE = zeros(num_L_T, num_x, num_strategies_pid);

    %% ========== 主循环 ==========
    fprintf('=== 曲线跟踪仿真 ===\n');
    fprintf('系统: K=%.1f, T=%.3f, u_lim=%.1f\n', K, T, u_lim);
    fprintf('==========================================\n');

    total = num_L_T * num_x;
    count = 0;

    for i = 1:num_L_T
        L_T = L_T_values(i);
        L = L_T * T;

        for j = 1:num_x
            x = x_values(j);
            lambda = x * T;

            count = count + 1;
            fprintf('[%d/%d] L/T=%.2f, x=%.1f\n', count, total, L_T, x);

            % PI参数
            Kp_pi = T / (K * (lambda + L));
            Ti_pi = T;
            Ki_pi = Kp_pi / Ti_pi;

            % PID参数
            Kp_pid = (T + 0.5*L) / (K * (lambda + 0.5*L));
            Ti_pid = T + 0.5*L;
            Ki_pid = Kp_pid / Ti_pid;
            Td_pid = T*L / (2*T + L);
            Kd_pid = Kp_pid * Td_pid;

            % PIDF参数（前馈增益）
            Kff = 1/K;  % 稳态前馈
            Kff_d = 0.01;  % 微分前馈

            % 存储时域数据
            y_pi_all = zeros(N, num_strategies_pi);
            u_sat_pi_all = zeros(N, num_strategies_pi);
            y_pid_all = zeros(N, num_strategies_pid);
            u_sat_pid_all = zeros(N, num_strategies_pid);
            y_pidf_all = zeros(N, num_strategies_pid);
            u_sat_pidf_all = zeros(N, num_strategies_pid);

            % 运行PI策略（6种）
            fprintf('  PI: ');
            for s = 1:num_strategies_pi
                strategy = strategies_pi{s};
                fprintf('%s ', strategy);
                [IAE_pi, ITAE_pi, y_pi_all(:, s), u_sat_pi_all(:, s)] = simulate_pi(...
                    K, T, L, Kp_pi, Ki_pi, Ts, N, w, u_lim, strategy);
                results.PI.IAE(i, j, s) = IAE_pi;
                results.PI.ITAE(i, j, s) = ITAE_pi;
            end
            fprintf('\n');

            % 运行PID策略（7种）
            fprintf('  PID: ');
            for s = 1:num_strategies_pid
                strategy = strategies_pid{s};
                fprintf('%s ', strategy);
                [IAE_pid, ITAE_pid, y_pid_all(:, s), u_sat_pid_all(:, s)] = simulate_pid(...
                    K, T, L, Kp_pid, Ki_pid, Kd_pid, Ts, N, w, u_lim, strategy);
                results.PID.IAE(i, j, s) = IAE_pid;
                results.PID.ITAE(i, j, s) = ITAE_pid;
            end
            fprintf('\n');

            % 运行PIDF策略（7种）
            fprintf('  PIDF: ');
            for s = 1:num_strategies_pid
                strategy = strategies_pid{s};
                fprintf('%s ', strategy);
                [IAE_pidf, ITAE_pidf, y_pidf_all(:, s), u_sat_pidf_all(:, s)] = simulate_pidf(...
                    K, T, L, Kp_pid, Ki_pid, Kd_pid, Kff, Kff_d, Ts, N, w, u_lim, strategy);
                results.PIDF.IAE(i, j, s) = IAE_pidf;
                results.PIDF.ITAE(i, j, s) = ITAE_pidf;
            end
            fprintf('\n');

            % 绘制时域图
            plot_time_domain_curves(t, w, L_T, x, ...
                y_pi_all, u_sat_pi_all, strategies_pi, ...
                y_pid_all, u_sat_pid_all, strategies_pid, ...
                y_pidf_all, u_sat_pidf_all, strategies_pid, ...
                u_lim, folders);
        end
    end

    %% ========== 绘制汇总图 ==========
    fprintf('\n绘制汇总图...\n');
    plot_summary(results, folders);

    % 保存结果
    save('scenario1_curve_results.mat', 'results');
    fprintf('完成！结果保存到 scenario1_curve_results.mat\n');

end

%% ========== 生成平滑曲线 ==========
function w = generate_smooth_curve(t, N)
    % 使用多个正弦波叠加生成复杂平滑曲线
    w_raw = zeros(N, 1);
    for k = 1:N
        w_raw(k) = 0.5 * sin(0.5*t(k)) ...        % 低频主分量
                 + 0.3 * sin(1.2*t(k) + 0.5) ...   % 中频分量
                 + 0.15 * sin(2.5*t(k) + 1.0) ...  % 高频分量
                 + 0.1 * cos(0.3*t(k));             % 缓慢漂移
    end
    % 归一化到 [0.1, 0.8] 范围
    w = w_raw - min(w_raw);
    w = w / max(w) * 0.7 + 0.1;
end

%% ========== PI仿真 ==========
function [IAE, ITAE, y, u_sat] = simulate_pi(K, T, L, Kp, Ki, Ts, N, w, u_lim, strategy)
    y = zeros(N, 1);
    u_c = zeros(N, 1);
    u_sat = zeros(N, 1);
    e = zeros(N, 1);
    e_sat = zeros(N, 1);
    u_i = zeros(N, 1);
    y_sys = zeros(N, 1);

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
        params.T = T;
        params.L = L;

        [u_c(k), u_sat(k), u_i(k), e_sat(k)] = PI(params);

        if k > delay_steps
            u_delayed = u_sat(k - delay_steps);
        else
            u_delayed = 0;
        end
        y_sys(k) = y_sys(k-1) + Ts * (K * u_delayed - y_sys(k-1)) / T;
        y(k) = y_sys(k);

        y_prev2 = y_prev;
        y_prev = y(k-1);
    end

    IAE = sum(abs(e)) * Ts;
    ITAE = sum((1:N)' .* abs(e)) * Ts;
end

%% ========== PID仿真 ==========
function [IAE, ITAE, y, u_sat] = simulate_pid(K, T, L, Kp, Ki, Kd, Ts, N, w, u_lim, strategy)
    y = zeros(N, 1);
    u_c = zeros(N, 1);
    u_sat = zeros(N, 1);
    e = zeros(N, 1);
    e_sat = zeros(N, 1);
    u_i = zeros(N, 1);
    u_d = zeros(N, 1);
    y_sys = zeros(N, 1);

    delay_steps = round(L / Ts);
    e_prev = 0;
    d_prev = 0;
    y_prev = 0;
    y_prev2 = 0;
    u_d_prev = 0;

    for k = 2:N
        e(k) = w(k) - y(k-1);

        params.e = e(k);
        params.e_prev = e_prev;
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

        [u_c(k), u_sat(k), u_i(k), e_sat(k), u_d(k)] = PID(params);

        if k > delay_steps
            u_delayed = u_sat(k - delay_steps);
        else
            u_delayed = 0;
        end
        y_sys(k) = y_sys(k-1) + Ts * (K * u_delayed - y_sys(k-1)) / T;
        y(k) = y_sys(k);

        e_prev = e(k);
        d_prev = u_d(k);
        y_prev2 = y_prev;
        y_prev = y(k-1);
        u_d_prev = u_d(k);
    end

    IAE = sum(abs(e)) * Ts;
    ITAE = sum((1:N)' .* abs(e)) * Ts;
end

%% ========== PIDF仿真 ==========
function [IAE, ITAE, y, u_sat] = simulate_pidf(K, T, L, Kp, Ki, Kd, Kff, Kff_d, Ts, N, w, u_lim, strategy)
    y = zeros(N, 1);
    u_c = zeros(N, 1);
    u_sat = zeros(N, 1);
    e = zeros(N, 1);
    e_sat = zeros(N, 1);
    u_i = zeros(N, 1);
    u_d = zeros(N, 1);
    u_ff = zeros(N, 1);
    y_sys = zeros(N, 1);

    delay_steps = round(L / Ts);
    e_prev = 0;
    d_prev = 0;
    y_prev = 0;
    y_prev2 = 0;
    u_d_prev = 0;
    w_prev = 0;

    for k = 2:N
        e(k) = w(k) - y(k-1);

        params.e = e(k);
        params.e_prev = e_prev;
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
        params.Kff = Kff;
        params.Kff_d = Kff_d;
        params.w_prev = w_prev;

        [u_c(k), u_sat(k), u_i(k), e_sat(k), u_d(k), u_ff(k)] = PIDF(params);

        if k > delay_steps
            u_delayed = u_sat(k - delay_steps);
        else
            u_delayed = 0;
        end
        y_sys(k) = y_sys(k-1) + Ts * (K * u_delayed - y_sys(k-1)) / T;
        y(k) = y_sys(k);

        e_prev = e(k);
        d_prev = u_d(k);
        y_prev2 = y_prev;
        y_prev = y(k-1);
        u_d_prev = u_d(k);
        w_prev = w(k);
    end

    IAE = sum(abs(e)) * Ts;
    ITAE = sum((1:N)' .* abs(e)) * Ts;
end

%% ========== 绘制时域图 ==========
function plot_time_domain_curves(t, w, L_T, x, ...
    y_pi, u_sat_pi, strategies_pi, ...
    y_pid, u_sat_pid, strategies_pid, ...
    y_pidf, u_sat_pidf, strategies_pid2, ...
    u_lim, folders)

    num_pi = length(strategies_pi);
    num_pid = length(strategies_pid);
    colors_pi = lines(num_pi);
    colors_pid = lines(num_pid);
    show_idx = t <= 15;  % 显示前15秒

    % PI - 输出图
    figure('Visible', 'off');
    hold on;
    plot(t(show_idx), w(show_idx), 'k--', 'LineWidth', 2, 'DisplayName', '设定值');
    for s = 1:num_pi
        plot(t(show_idx), y_pi(show_idx, s), '-', 'Color', colors_pi(s,:), ...
            'LineWidth', 1.5, 'DisplayName', strategies_pi{s});
    end
    xlabel('时间 (s)'); ylabel('输出 y');
    title(sprintf('PI 输出 (L/T=%.2f, x=%.1f)', L_T, x));
    legend('Location', 'best', 'FontSize', 8); grid on; hold off;
    saveas(gcf, fullfile(folders{1}, sprintf('y_LT%.2f_x%.1f.png', L_T, x)));

    % PI - 控制器图
    figure('Visible', 'off');
    hold on;
    yline(u_lim, 'k--', 'LineWidth', 1, 'DisplayName', 'u_{lim}');
    yline(-u_lim, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
    for s = 1:num_pi
        plot(t(show_idx), u_sat_pi(show_idx, s), '-', 'Color', colors_pi(s,:), ...
            'LineWidth', 1.5, 'DisplayName', strategies_pi{s});
    end
    xlabel('时间 (s)'); ylabel('u_{sat}');
    title(sprintf('PI 控制器 (L/T=%.2f, x=%.1f)', L_T, x));
    legend('Location', 'best', 'FontSize', 8); grid on; ylim([-1.5, 1.5]); hold off;
    saveas(gcf, fullfile(folders{1}, sprintf('u_LT%.2f_x%.1f.png', L_T, x)));

    % PID - 输出图
    figure('Visible', 'off');
    hold on;
    plot(t(show_idx), w(show_idx), 'k--', 'LineWidth', 2, 'DisplayName', '设定值');
    for s = 1:num_pid
        plot(t(show_idx), y_pid(show_idx, s), '-', 'Color', colors_pid(s,:), ...
            'LineWidth', 1.5, 'DisplayName', strategies_pid{s});
    end
    xlabel('时间 (s)'); ylabel('输出 y');
    title(sprintf('PID 输出 (L/T=%.2f, x=%.1f)', L_T, x));
    legend('Location', 'best', 'FontSize', 8); grid on; hold off;
    saveas(gcf, fullfile(folders{2}, sprintf('y_LT%.2f_x%.1f.png', L_T, x)));

    % PID - 控制器图
    figure('Visible', 'off');
    hold on;
    yline(u_lim, 'k--', 'LineWidth', 1, 'DisplayName', 'u_{lim}');
    yline(-u_lim, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
    for s = 1:num_pid
        plot(t(show_idx), u_sat_pid(show_idx, s), '-', 'Color', colors_pid(s,:), ...
            'LineWidth', 1.5, 'DisplayName', strategies_pid{s});
    end
    xlabel('时间 (s)'); ylabel('u_{sat}');
    title(sprintf('PID 控制器 (L/T=%.2f, x=%.1f)', L_T, x));
    legend('Location', 'best', 'FontSize', 8); grid on; ylim([-1.5, 1.5]); hold off;
    saveas(gcf, fullfile(folders{2}, sprintf('u_LT%.2f_x%.1f.png', L_T, x)));

    % PIDF - 输出图
    figure('Visible', 'off');
    hold on;
    plot(t(show_idx), w(show_idx), 'k--', 'LineWidth', 2, 'DisplayName', '设定值');
    for s = 1:num_pid
        plot(t(show_idx), y_pidf(show_idx, s), '-', 'Color', colors_pid(s,:), ...
            'LineWidth', 1.5, 'DisplayName', strategies_pid{s});
    end
    xlabel('时间 (s)'); ylabel('输出 y');
    title(sprintf('PIDF 输出 (L/T=%.2f, x=%.1f)', L_T, x));
    legend('Location', 'best', 'FontSize', 8); grid on; hold off;
    saveas(gcf, fullfile(folders{3}, sprintf('y_LT%.2f_x%.1f.png', L_T, x)));

    % PIDF - 控制器图
    figure('Visible', 'off');
    hold on;
    yline(u_lim, 'k--', 'LineWidth', 1, 'DisplayName', 'u_{lim}');
    yline(-u_lim, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
    for s = 1:num_pid
        plot(t(show_idx), u_sat_pidf(show_idx, s), '-', 'Color', colors_pid(s,:), ...
            'LineWidth', 1.5, 'DisplayName', strategies_pid{s});
    end
    xlabel('时间 (s)'); ylabel('u_{sat}');
    title(sprintf('PIDF 控制器 (L/T=%.2f, x=%.1f)', L_T, x));
    legend('Location', 'best', 'FontSize', 8); grid on; ylim([-1.5, 1.5]); hold off;
    saveas(gcf, fullfile(folders{3}, sprintf('u_LT%.2f_x%.1f.png', L_T, x)));

    close all;
end

%% ========== 绘制汇总图 ==========
function plot_summary(results, folders)
    num_L_T = length(results.L_T_values);
    num_x = length(results.x_values);

    % PI使用6种策略
    strategies_pi = results.strategies_pi;
    num_pi = length(strategies_pi);

    % PID/PIDF使用7种策略
    strategies_pid = results.strategies_pid;
    num_pid = length(strategies_pid);

    % PI汇总图
    for i = 1:num_L_T
        for j = 1:num_x
            figure('Visible', 'off');
            IAE_vals = squeeze(results.PI.IAE(i, j, :));
            bar(IAE_vals);
            set(gca, 'XTickLabel', strategies_pi, 'XTickLabelRotation', 45);
            ylabel('IAE');
            title(sprintf('PI IAE (L/T=%.2f, x=%.1f)', results.L_T_values(i), results.x_values(j)));
            grid on;
            saveas(gcf, fullfile(folders{1}, sprintf('PI_IAE_LT%.2f_x%.1f.png', results.L_T_values(i), results.x_values(j))));

            figure('Visible', 'off');
            ITAE_vals = squeeze(results.PI.ITAE(i, j, :));
            bar(ITAE_vals);
            set(gca, 'XTickLabel', strategies_pi, 'XTickLabelRotation', 45);
            ylabel('ITAE');
            title(sprintf('PI ITAE (L/T=%.2f, x=%.1f)', results.L_T_values(i), results.x_values(j)));
            grid on;
            saveas(gcf, fullfile(folders{1}, sprintf('PI_ITAE_LT%.2f_x%.1f.png', results.L_T_values(i), results.x_values(j))));
        end
    end

    % PID汇总图
    for i = 1:num_L_T
        for j = 1:num_x
            figure('Visible', 'off');
            IAE_vals = squeeze(results.PID.IAE(i, j, :));
            bar(IAE_vals);
            set(gca, 'XTickLabel', strategies_pid, 'XTickLabelRotation', 45);
            ylabel('IAE');
            title(sprintf('PID IAE (L/T=%.2f, x=%.1f)', results.L_T_values(i), results.x_values(j)));
            grid on;
            saveas(gcf, fullfile(folders{2}, sprintf('PID_IAE_LT%.2f_x%.1f.png', results.L_T_values(i), results.x_values(j))));

            figure('Visible', 'off');
            ITAE_vals = squeeze(results.PID.ITAE(i, j, :));
            bar(ITAE_vals);
            set(gca, 'XTickLabel', strategies_pid, 'XTickLabelRotation', 45);
            ylabel('ITAE');
            title(sprintf('PID ITAE (L/T=%.2f, x=%.1f)', results.L_T_values(i), results.x_values(j)));
            grid on;
            saveas(gcf, fullfile(folders{2}, sprintf('PID_ITAE_LT%.2f_x%.1f.png', results.L_T_values(i), results.x_values(j))));
        end
    end

    % PIDF汇总图
    for i = 1:num_L_T
        for j = 1:num_x
            figure('Visible', 'off');
            IAE_vals = squeeze(results.PIDF.IAE(i, j, :));
            bar(IAE_vals);
            set(gca, 'XTickLabel', strategies_pid, 'XTickLabelRotation', 45);
            ylabel('IAE');
            title(sprintf('PIDF IAE (L/T=%.2f, x=%.1f)', results.L_T_values(i), results.x_values(j)));
            grid on;
            saveas(gcf, fullfile(folders{3}, sprintf('PIDF_IAE_LT%.2f_x%.1f.png', results.L_T_values(i), results.x_values(j))));

            figure('Visible', 'off');
            ITAE_vals = squeeze(results.PIDF.ITAE(i, j, :));
            bar(ITAE_vals);
            set(gca, 'XTickLabel', strategies_pid, 'XTickLabelRotation', 45);
            ylabel('ITAE');
            title(sprintf('PIDF ITAE (L/T=%.2f, x=%.1f)', results.L_T_values(i), results.x_values(j)));
            grid on;
            saveas(gcf, fullfile(folders{3}, sprintf('PIDF_ITAE_LT%.2f_x%.1f.png', results.L_T_values(i), results.x_values(j))));
        end
    end

    % 绘制设定值曲线
    figure('Visible', 'off');
    plot(results.t, results.w, 'b-', 'LineWidth', 2);
    xlabel('时间 (s)');
    ylabel('设定值 w');
    title('曲线跟踪设定值');
    grid on;
    saveas(gcf, fullfile(folders{1}, 'setpoint_curve.png'));

    close all;
end
