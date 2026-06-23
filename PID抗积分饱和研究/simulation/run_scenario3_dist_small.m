%% 情景三：平衡pitch环 - 抗持续小幅扰动
% 作者：Luciano
% 日期：2026-06-09
% 描述：测试控制器抵抗持续小幅扰动的能力
%       目标：保持输出在0
%       输出：3_1(PI), 3_3(PID), 3_5(PIDF)

function results = run_scenario3_dist_small()
    clc; clear; close all;

    %% ========== 系统参数 ==========
    K = 1;
    a2 = 0.04;
    a1 = 0.1;
    a0 = 1;
    T_eq = 0.05;
    u_lim = 1;

    t_sim = 30;
    Ts = 0.001;
    N = t_sim / Ts;
    t = (0:N-1) * Ts;

    L_T_values = [0.02, 0.1, 0.3, 0.5];
    x_values = [0.2, 0.5, 0.8];

    strategies_pi = {'DBC_CLA', 'IBC', 'CI', 'CBC1', 'CBC2', 'DBC_STR'};
    strategies_pid = {'DBC_CLA', 'IBC', 'CI', 'CBC1', 'CBC2', 'DBC_STR', 'GBC'};
    num_pi = length(strategies_pi);
    num_pid = length(strategies_pid);

    folders = {'3_1', '3_3', '3_5'};
    for f = 1:length(folders)
        if ~exist(folders{f}, 'dir'), mkdir(folders{f}); end
    end

    %% 生成持续小幅扰动
    % 正弦扰动 + 随机噪声
    d = 0.1 * sin(2*pi*0.5*t)' + 0.05 * sin(2*pi*1.5*t)' + 0.02 * randn(N, 1);
    w = zeros(N, 1);  % 设定值为0

    %% 初始化
    num_L_T = length(L_T_values);
    num_x = length(x_values);

    results.L_T_values = L_T_values;
    results.x_values = x_values;
    results.strategies_pi = strategies_pi;
    results.strategies_pid = strategies_pid;
    results.t = t;
    results.w = w;
    results.d = d;

    results.PI.IAE = zeros(num_L_T, num_x, num_pi);
    results.PI.ITAE = zeros(num_L_T, num_x, num_pi);
    results.PID.IAE = zeros(num_L_T, num_x, num_pid);
    results.PID.ITAE = zeros(num_L_T, num_x, num_pid);
    results.PIDF.IAE = zeros(num_L_T, num_x, num_pid);
    results.PIDF.ITAE = zeros(num_L_T, num_x, num_pid);

    %% ========== 主循环 ==========
    fprintf('=== 情景三 抗持续小幅扰动 ===\n');
    fprintf('扰动: 0.1*sin(πt) + 0.05*sin(3πt) + 噪声\n');
    fprintf('==========================================\n');

    T_eff = a2/a1;
    total = num_L_T * num_x;
    count = 0;

    for i = 1:num_L_T
        L_T = L_T_values(i);
        L = L_T * T_eq;

        for j = 1:num_x
            x = x_values(j);
            lambda = x * T_eq;

            count = count + 1;
            fprintf('[%d/%d] L/T=%.2f, x=%.1f\n', count, total, L_T, x);

            % PI参数
            Kp_pi = T_eq / (K * (lambda + L));
            Ti_pi = T_eq;
            Ki_pi = Kp_pi / Ti_pi;

            % PID参数
            Kp_pid = (T_eq + 0.5*L) / (K * (lambda + 0.5*L));
            Ti_pid = T_eq + 0.5*L;
            Ki_pid = Kp_pid / Ti_pid;
            Td_pid = T_eq*L / (2*T_eq + L);
            Kd_pid = Kp_pid * Td_pid;

            % PIDF参数
            Kff = 0;
            Kff_d = 0;

            % PI仿真
            fprintf('  PI: ');
            for s = 1:num_pi
                fprintf('%s ', strategies_pi{s});
                [results.PI.IAE(i,j,s), results.PI.ITAE(i,j,s)] = ...
                    simulate_dist(K, a2, a1, a0, L, Kp_pi, Ki_pi, 0, Ts, N, w, d, u_lim, strategies_pi{s}, T_eff, 'PI');
            end
            fprintf('\n');

            % PID仿真
            fprintf('  PID: ');
            for s = 1:num_pid
                fprintf('%s ', strategies_pid{s});
                [results.PID.IAE(i,j,s), results.PID.ITAE(i,j,s)] = ...
                    simulate_dist(K, a2, a1, a0, L, Kp_pid, Ki_pid, Kd_pid, Ts, N, w, d, u_lim, strategies_pid{s}, T_eff, 'PID');
            end
            fprintf('\n');

            % PIDF仿真
            fprintf('  PIDF: ');
            for s = 1:num_pid
                fprintf('%s ', strategies_pid{s});
                [results.PIDF.IAE(i,j,s), results.PIDF.ITAE(i,j,s)] = ...
                    simulate_dist(K, a2, a1, a0, L, Kp_pid, Ki_pid, Kd_pid, Ts, N, w, d, u_lim, strategies_pid{s}, T_eff, 'PIDF', Kff, Kff_d);
            end
            fprintf('\n');
        end
    end

    %% ========== 绘图 ==========
    fprintf('\n绘图...\n');

    % 绘制柱状图
    plot_bars(results, folders);

    % 绘制时域图（选择代表性组合）
    plot_time_domain(results, K, a2, a1, a0, T_eq, Ts, N, w, d, u_lim, T_eff, folders);

    save('scenario3_dist_small_results.mat', 'results');
    fprintf('完成！\n');
end

%% ========== 仿真函数 ==========
function [IAE, ITAE] = simulate_dist(K, a2, a1, a0, L, Kp, Ki, Kd, Ts, N, w, d, u_lim, strategy, T_eff, ctrl_type, Kff, Kff_d)
    if nargin < 17, Kff = 0; Kff_d = 0; end

    y = zeros(N,1); u_c = zeros(N,1); u_sat = zeros(N,1);
    e = zeros(N,1); e_sat = zeros(N,1); u_i = zeros(N,1); u_d = zeros(N,1);
    y1 = zeros(N,1); y2 = zeros(N,1);

    delay_steps = round(L/Ts);
    e_prev = 0; d_prev = 0; y_prev = 0; y_prev2 = 0; u_d_prev = 0; w_prev = 0;

    for k = 2:N
        e(k) = w(k) - y(k-1);

        params.e = e(k); params.e_prev = e_prev; params.e_sat_prev = e_sat(k-1);
        params.Kp = Kp; params.Ki = Ki; params.Kd = Kd; params.Ts = Ts;
        params.u_i_prev = u_i(k-1); params.u_c_prev = u_c(max(k-1,1));
        params.u_sat_prev = u_sat(max(k-1,1)); params.u_lim = u_lim;
        params.strategy = strategy; params.N_filter = 10; params.d_prev = d_prev;
        params.y = y(max(k-1,1)); params.y_prev = y_prev; params.y_prev2 = y_prev2;
        params.w = w(k); params.K = K; params.T = T_eff; params.L = L;
        params.u_d_prev = u_d_prev; params.Kff = Kff; params.Kff_d = Kff_d; params.w_prev = w_prev;

        switch ctrl_type
            case 'PI'
                [u_c(k), u_sat(k), u_i(k), e_sat(k)] = PI(params);
                u_d(k) = 0;
            case 'PID'
                [u_c(k), u_sat(k), u_i(k), e_sat(k), u_d(k)] = PID(params);
            case 'PIDF'
                [u_c(k), u_sat(k), u_i(k), e_sat(k), u_d(k)] = PIDF(params);
        end

        if k > delay_steps, u_delayed = u_sat(k-delay_steps); else, u_delayed = 0; end

        % 带扰动的系统响应
        dy1 = y2(k-1);
        dy2 = (K * u_delayed + d(k) - a1*y2(k-1) - a0*y1(k-1)) / a2;
        y1(k) = y1(k-1) + Ts*dy1;
        y2(k) = y2(k-1) + Ts*dy2;
        y(k) = y1(k);

        e_prev = e(k); d_prev = u_d(k); y_prev2 = y_prev; y_prev = y(k-1);
        u_d_prev = u_d(k); w_prev = w(k);
    end

    IAE = sum(abs(e))*Ts;
    ITAE = sum((1:N)'.*abs(e))*Ts;
end

%% ========== 柱状图 ==========
function plot_bars(results, folders)
    num_L_T = length(results.L_T_values);
    num_x = length(results.x_values);

    for i = 1:num_L_T
        for j = 1:num_x
            % PI
            figure('Visible','off');
            subplot(2,1,1); bar(squeeze(results.PI.IAE(i,j,:)));
            set(gca,'XTickLabel',results.strategies_pi,'XTickLabelRotation',45);
            ylabel('IAE'); title(sprintf('PI 抗扰 (L/T=%.2f, x=%.1f)', results.L_T_values(i), results.x_values(j)));
            grid on;

            subplot(2,1,2); bar(squeeze(results.PI.ITAE(i,j,:)));
            set(gca,'XTickLabel',results.strategies_pi,'XTickLabelRotation',45);
            ylabel('ITAE'); grid on;
            saveas(gcf, fullfile(folders{1}, sprintf('PI_scores_LT%.2f_x%.1f.png', results.L_T_values(i), results.x_values(j))));

            % PID
            figure('Visible','off');
            subplot(2,1,1); bar(squeeze(results.PID.IAE(i,j,:)));
            set(gca,'XTickLabel',results.strategies_pid,'XTickLabelRotation',45);
            ylabel('IAE'); title(sprintf('PID 抗扰 (L/T=%.2f, x=%.1f)', results.L_T_values(i), results.x_values(j)));
            grid on;

            subplot(2,1,2); bar(squeeze(results.PID.ITAE(i,j,:)));
            set(gca,'XTickLabel',results.strategies_pid,'XTickLabelRotation',45);
            ylabel('ITAE'); grid on;
            saveas(gcf, fullfile(folders{2}, sprintf('PID_scores_LT%.2f_x%.1f.png', results.L_T_values(i), results.x_values(j))));

            % PIDF
            figure('Visible','off');
            subplot(2,1,1); bar(squeeze(results.PIDF.IAE(i,j,:)));
            set(gca,'XTickLabel',results.strategies_pid,'XTickLabelRotation',45);
            ylabel('IAE'); title(sprintf('PIDF 抗扰 (L/T=%.2f, x=%.1f)', results.L_T_values(i), results.x_values(j)));
            grid on;

            subplot(2,1,2); bar(squeeze(results.PIDF.ITAE(i,j,:)));
            set(gca,'XTickLabel',results.strategies_pid,'XTickLabelRotation',45);
            ylabel('ITAE'); grid on;
            saveas(gcf, fullfile(folders{3}, sprintf('PIDF_scores_LT%.2f_x%.1f.png', results.L_T_values(i), results.x_values(j))));
        end
    end
    close all;
end

%% ========== 时域图 ==========
function plot_time_domain(results, K, a2, a1, a0, T_eq, Ts, N, w, d, u_lim, T_eff, folders)
    % 选择代表性组合：L/T=0.1, x=0.5
    i = 2; j = 2;
    L_T = results.L_T_values(i);
    L = L_T * T_eq;
    x = results.x_values(j);
    lambda = x * T_eq;
    t = results.t;

    strategies_pi = results.strategies_pi;
    strategies_pid = results.strategies_pid;
    num_pi = length(strategies_pi);
    num_pid = length(strategies_pid);
    colors_pi = lines(num_pi);
    colors_pid = lines(num_pid);

    Kp_pi = T_eq / (K * (lambda + L));
    Ti_pi = T_eq; Ki_pi = Kp_pi / Ti_pi;

    Kp_pid = (T_eq + 0.5*L) / (K * (lambda + 0.5*L));
    Ti_pid = T_eq + 0.5*L; Ki_pid = Kp_pid / Ti_pid;
    Td_pid = T_eq*L / (2*T_eq + L); Kd_pid = Kp_pid * Td_pid;

    show_idx = t <= 10;

    % PI时域图
    figure('Visible','off');
    subplot(2,1,1); hold on;
    plot(t(show_idx), w(show_idx), 'k--', 'LineWidth', 1.5, 'DisplayName', 'w');
    for s = 1:num_pi
        [IAE, ITAE, y] = simulate_dist_full(K, a2, a1, a0, L, Kp_pi, Ki_pi, 0, Ts, N, w, d, u_lim, strategies_pi{s}, T_eff, 'PI');
        plot(t(show_idx), y(show_idx), '-', 'Color', colors_pi(s,:), 'LineWidth', 1.5, 'DisplayName', sprintf('%s (%.2f)', strategies_pi{s}, IAE));
    end
    xlabel('时间 (s)'); ylabel('输出 y'); title(sprintf('PI 输出 (L/T=%.2f, x=%.1f)', L_T, x));
    legend('Location','best','FontSize',7); grid on; hold off;

    subplot(2,1,2); hold on;
    plot(t(show_idx), d(show_idx), 'k-', 'LineWidth', 1, 'DisplayName', '扰动');
    for s = 1:num_pi
        [~, ~, ~, u_sat] = simulate_dist_full(K, a2, a1, a0, L, Kp_pi, Ki_pi, 0, Ts, N, w, d, u_lim, strategies_pi{s}, T_eff, 'PI');
        plot(t(show_idx), u_sat(show_idx), '-', 'Color', colors_pi(s,:), 'LineWidth', 1.5, 'DisplayName', strategies_pi{s});
    end
    xlabel('时间 (s)'); ylabel('u_{sat}'); title('PI 控制器');
    legend('Location','best','FontSize',7); grid on; hold off;
    saveas(gcf, fullfile(folders{1}, sprintf('PI_time_LT%.2f_x%.1f.png', L_T, x)));

    % PID时域图
    figure('Visible','off');
    subplot(2,1,1); hold on;
    plot(t(show_idx), w(show_idx), 'k--', 'LineWidth', 1.5, 'DisplayName', 'w');
    for s = 1:num_pid
        [IAE, ITAE, y] = simulate_dist_full(K, a2, a1, a0, L, Kp_pid, Ki_pid, Kd_pid, Ts, N, w, d, u_lim, strategies_pid{s}, T_eff, 'PID');
        plot(t(show_idx), y(show_idx), '-', 'Color', colors_pid(s,:), 'LineWidth', 1.5, 'DisplayName', sprintf('%s (%.2f)', strategies_pid{s}, IAE));
    end
    xlabel('时间 (s)'); ylabel('输出 y'); title(sprintf('PID 输出 (L/T=%.2f, x=%.1f)', L_T, x));
    legend('Location','best','FontSize',7); grid on; hold off;

    subplot(2,1,2); hold on;
    plot(t(show_idx), d(show_idx), 'k-', 'LineWidth', 1, 'DisplayName', '扰动');
    for s = 1:num_pid
        [~, ~, ~, u_sat] = simulate_dist_full(K, a2, a1, a0, L, Kp_pid, Ki_pid, Kd_pid, Ts, N, w, d, u_lim, strategies_pid{s}, T_eff, 'PID');
        plot(t(show_idx), u_sat(show_idx), '-', 'Color', colors_pid(s,:), 'LineWidth', 1.5, 'DisplayName', strategies_pid{s});
    end
    xlabel('时间 (s)'); ylabel('u_{sat}'); title('PID 控制器');
    legend('Location','best','FontSize',7); grid on; hold off;
    saveas(gcf, fullfile(folders{2}, sprintf('PID_time_LT%.2f_x%.1f.png', L_T, x)));

    % PIDF时域图
    figure('Visible','off');
    subplot(2,1,1); hold on;
    plot(t(show_idx), w(show_idx), 'k--', 'LineWidth', 1.5, 'DisplayName', 'w');
    for s = 1:num_pid
        [IAE, ITAE, y] = simulate_dist_full(K, a2, a1, a0, L, Kp_pid, Ki_pid, Kd_pid, Ts, N, w, d, u_lim, strategies_pid{s}, T_eff, 'PIDF', 0, 0);
        plot(t(show_idx), y(show_idx), '-', 'Color', colors_pid(s,:), 'LineWidth', 1.5, 'DisplayName', sprintf('%s (%.2f)', strategies_pid{s}, IAE));
    end
    xlabel('时间 (s)'); ylabel('输出 y'); title(sprintf('PIDF 输出 (L/T=%.2f, x=%.1f)', L_T, x));
    legend('Location','best','FontSize',7); grid on; hold off;

    subplot(2,1,2); hold on;
    plot(t(show_idx), d(show_idx), 'k-', 'LineWidth', 1, 'DisplayName', '扰动');
    for s = 1:num_pid
        [~, ~, ~, u_sat] = simulate_dist_full(K, a2, a1, a0, L, Kp_pid, Ki_pid, Kd_pid, Ts, N, w, d, u_lim, strategies_pid{s}, T_eff, 'PIDF', 0, 0);
        plot(t(show_idx), u_sat(show_idx), '-', 'Color', colors_pid(s,:), 'LineWidth', 1.5, 'DisplayName', strategies_pid{s});
    end
    xlabel('时间 (s)'); ylabel('u_{sat}'); title('PIDF 控制器');
    legend('Location','best','FontSize',7); grid on; hold off;
    saveas(gcf, fullfile(folders{3}, sprintf('PIDF_time_LT%.2f_x%.1f.png', L_T, x)));

    close all;
end

%% ========== 完整仿真函数（返回时域数据） ==========
function [IAE, ITAE, y, u_sat] = simulate_dist_full(K, a2, a1, a0, L, Kp, Ki, Kd, Ts, N, w, d, u_lim, strategy, T_eff, ctrl_type, Kff, Kff_d)
    if nargin < 17, Kff = 0; Kff_d = 0; end

    y = zeros(N,1); u_c = zeros(N,1); u_sat = zeros(N,1);
    e = zeros(N,1); e_sat = zeros(N,1); u_i = zeros(N,1); u_d = zeros(N,1);
    y1 = zeros(N,1); y2 = zeros(N,1);

    delay_steps = round(L/Ts);
    e_prev = 0; d_prev = 0; y_prev = 0; y_prev2 = 0; u_d_prev = 0; w_prev = 0;

    for k = 2:N
        e(k) = w(k) - y(k-1);
        params.e = e(k); params.e_prev = e_prev; params.e_sat_prev = e_sat(k-1);
        params.Kp = Kp; params.Ki = Ki; params.Kd = Kd; params.Ts = Ts;
        params.u_i_prev = u_i(k-1); params.u_c_prev = u_c(max(k-1,1));
        params.u_sat_prev = u_sat(max(k-1,1)); params.u_lim = u_lim;
        params.strategy = strategy; params.N_filter = 10; params.d_prev = d_prev;
        params.y = y(max(k-1,1)); params.y_prev = y_prev; params.y_prev2 = y_prev2;
        params.w = w(k); params.K = K; params.T = T_eff; params.L = L;
        params.u_d_prev = u_d_prev; params.Kff = Kff; params.Kff_d = Kff_d; params.w_prev = w_prev;

        switch ctrl_type
            case 'PI', [u_c(k), u_sat(k), u_i(k), e_sat(k)] = PI(params); u_d(k) = 0;
            case 'PID', [u_c(k), u_sat(k), u_i(k), e_sat(k), u_d(k)] = PID(params);
            case 'PIDF', [u_c(k), u_sat(k), u_i(k), e_sat(k), u_d(k)] = PIDF(params);
        end

        if k > delay_steps, u_delayed = u_sat(k-delay_steps); else, u_delayed = 0; end
        dy1 = y2(k-1); dy2 = (K*u_delayed + d(k) - a1*y2(k-1) - a0*y1(k-1))/a2;
        y1(k) = y1(k-1) + Ts*dy1; y2(k) = y2(k-1) + Ts*dy2; y(k) = y1(k);
        e_prev = e(k); d_prev = u_d(k); y_prev2 = y_prev; y_prev = y(k-1);
        u_d_prev = u_d(k); w_prev = w(k);
    end

    IAE = sum(abs(e))*Ts; ITAE = sum((1:N)'.*abs(e))*Ts;
end
