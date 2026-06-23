%% 情景三：平衡pitch环 - 曲线跟踪仿真
% 作者：Luciano
% 日期：2026-06-09
% 描述：二阶振荡系统曲线跟踪，比较PI/PID/PIDF控制器
%       输出：3_3(PI), 3_4(PID), 3_5(PIDF)

function results = run_scenario3_curve()
    clc; clear; close all;

    %% ========== 系统参数 ==========
    K = 1;
    a2 = 0.04;
    a1 = 0.1;
    a0 = 1;
    T_eq = 0.05;
    u_lim = 1;

    omega_n = sqrt(a0 / a2);
    zeta = a1 / (2 * sqrt(a0 * a2));

    t_sim = 30;
    Ts = 0.001;
    N = t_sim / Ts;
    t = (0:N-1) * Ts;

    L_T_values = [0.02, 0.1, 0.3, 0.5];
    x_values = [0.2, 0.5, 0.8];

    strategies_pi = {'DBC_CLA', 'IBC', 'CI', 'CBC1', 'CBC2', 'DBC_STR'};
    strategies_pid = {'DBC_CLA', 'IBC', 'CI', 'CBC1', 'CBC2', 'DBC_STR', 'GBC'};
    num_strategies_pi = length(strategies_pi);
    num_strategies_pid = length(strategies_pid);

    folders = {'3_3', '3_4', '3_5'};
    for f = 1:length(folders)
        if ~exist(folders{f}, 'dir'), mkdir(folders{f}); end
    end

    %% 生成平滑曲线
    w = generate_smooth_curve(t, N);

    %% 初始化
    num_L_T = length(L_T_values);
    num_x = length(x_values);

    results.L_T_values = L_T_values;
    results.x_values = x_values;
    results.strategies_pi = strategies_pi;
    results.strategies_pid = strategies_pid;
    results.t = t;
    results.w = w;

    results.PI.IAE = zeros(num_L_T, num_x, num_strategies_pi);
    results.PI.ITAE = zeros(num_L_T, num_x, num_strategies_pi);
    results.PID.IAE = zeros(num_L_T, num_x, num_strategies_pid);
    results.PID.ITAE = zeros(num_L_T, num_x, num_strategies_pid);
    results.PIDF.IAE = zeros(num_L_T, num_x, num_strategies_pid);
    results.PIDF.ITAE = zeros(num_L_T, num_x, num_strategies_pid);

    %% ========== 主循环 ==========
    fprintf('=== 情景三 曲线跟踪仿真 ===\n');
    fprintf('系统: G4(s) = 1/(0.04s^2 + 0.1s + 1) * e^(-Ls)\n');
    fprintf('omega_n = %.2f rad/s, zeta = %.3f\n', omega_n, zeta);
    fprintf('==========================================\n');

    total = num_L_T * num_x;
    count = 0;
    T_eff = a2/a1;

    for i = 1:num_L_T
        L_T = L_T_values(i);
        L = L_T * T_eq;

        for j = 1:num_x
            x = x_values(j);
            lambda = x * T_eq;

            count = count + 1;
            fprintf('[%d/%d] L/T=%.2f, x=%.1f\n', count, total, L_T, x);

            Kp_pi = T_eq / (K * (lambda + L));
            Ti_pi = T_eq;
            Ki_pi = Kp_pi / Ti_pi;

            Kp_pid = (T_eq + 0.5*L) / (K * (lambda + 0.5*L));
            Ti_pid = T_eq + 0.5*L;
            Ki_pid = Kp_pid / Ti_pid;
            Td_pid = T_eq*L / (2*T_eq + L);
            Kd_pid = Kp_pid * Td_pid;

            Kff = 1/K;
            Kff_d = 0.01;

            y_pi_all = zeros(N, num_strategies_pi);
            u_sat_pi_all = zeros(N, num_strategies_pi);
            y_pid_all = zeros(N, num_strategies_pid);
            u_sat_pid_all = zeros(N, num_strategies_pid);
            y_pidf_all = zeros(N, num_strategies_pid);
            u_sat_pidf_all = zeros(N, num_strategies_pid);

            fprintf('  PI: ');
            for s = 1:num_strategies_pi
                fprintf('%s ', strategies_pi{s});
                [results.PI.IAE(i,j,s), results.PI.ITAE(i,j,s), y_pi_all(:,s), u_sat_pi_all(:,s)] = ...
                    simulate_pi_osc(K, a2, a1, a0, L, Kp_pi, Ki_pi, Ts, N, w, u_lim, strategies_pi{s}, T_eff);
            end
            fprintf('\n');

            fprintf('  PID: ');
            for s = 1:num_strategies_pid
                fprintf('%s ', strategies_pid{s});
                [results.PID.IAE(i,j,s), results.PID.ITAE(i,j,s), y_pid_all(:,s), u_sat_pid_all(:,s)] = ...
                    simulate_pid_osc(K, a2, a1, a0, L, Kp_pid, Ki_pid, Kd_pid, Ts, N, w, u_lim, strategies_pid{s}, T_eff);
            end
            fprintf('\n');

            fprintf('  PIDF: ');
            for s = 1:num_strategies_pid
                fprintf('%s ', strategies_pid{s});
                [results.PIDF.IAE(i,j,s), results.PIDF.ITAE(i,j,s), y_pidf_all(:,s), u_sat_pidf_all(:,s)] = ...
                    simulate_pidf_osc(K, a2, a1, a0, L, Kp_pid, Ki_pid, Kd_pid, Kff, Kff_d, Ts, N, w, u_lim, strategies_pid{s}, T_eff);
            end
            fprintf('\n');

            plot_time_domain_curves(t, w, L_T, x, ...
                y_pi_all, u_sat_pi_all, strategies_pi, ...
                y_pid_all, u_sat_pid_all, strategies_pid, ...
                y_pidf_all, u_sat_pidf_all, u_lim, folders);
        end
    end

    fprintf('\n绘制汇总图...\n');
    plot_summary(results, folders);

    save('scenario3_curve_results.mat', 'results');
    fprintf('完成！结果保存到 scenario3_curve_results.mat\n');
end

%% ========== 生成平滑曲线 ==========
function w = generate_smooth_curve(t, N)
    w_raw = zeros(N, 1);
    for k = 1:N
        w_raw(k) = 0.5*sin(0.5*t(k)) + 0.3*sin(1.2*t(k)+0.5) + 0.15*sin(2.5*t(k)+1.0) + 0.1*cos(0.3*t(k));
    end
    w = w_raw - min(w_raw);
    w = w / max(w) * 0.7 + 0.1;
end

%% ========== PI仿真 ==========
function [IAE, ITAE, y, u_sat] = simulate_pi_osc(K, a2, a1, a0, L, Kp, Ki, Ts, N, w, u_lim, strategy, T_eff)
    y = zeros(N,1); u_c = zeros(N,1); u_sat = zeros(N,1);
    e = zeros(N,1); e_sat = zeros(N,1); u_i = zeros(N,1);
    y1 = zeros(N,1); y2 = zeros(N,1);
    delay_steps = round(L/Ts); y_prev = 0; y_prev2 = 0;

    for k = 2:N
        e(k) = w(k) - y(k-1);
        params.e = e(k); params.e_prev = e(max(k-1,1)); params.e_sat_prev = e_sat(k-1);
        params.Kp = Kp; params.Ki = Ki; params.Ts = Ts; params.u_i_prev = u_i(k-1);
        params.u_c_prev = u_c(max(k-1,1)); params.u_sat_prev = u_sat(max(k-1,1));
        params.u_lim = u_lim; params.strategy = strategy;
        params.y = y(max(k-1,1)); params.y_prev = y_prev; params.y_prev2 = y_prev2;
        params.w = w(k); params.K = K; params.T = T_eff; params.L = L;
        [u_c(k), u_sat(k), u_i(k), e_sat(k)] = PI(params);

        if k > delay_steps, u_delayed = u_sat(k-delay_steps); else, u_delayed = 0; end
        dy1 = y2(k-1); dy2 = (K*u_delayed - a1*y2(k-1) - a0*y1(k-1))/a2;
        y1(k) = y1(k-1) + Ts*dy1; y2(k) = y2(k-1) + Ts*dy2; y(k) = y1(k);
        y_prev2 = y_prev; y_prev = y(k-1);
    end
    IAE = sum(abs(e))*Ts; ITAE = sum((1:N)'.*abs(e))*Ts;
end

%% ========== PID仿真 ==========
function [IAE, ITAE, y, u_sat] = simulate_pid_osc(K, a2, a1, a0, L, Kp, Ki, Kd, Ts, N, w, u_lim, strategy, T_eff)
    y = zeros(N,1); u_c = zeros(N,1); u_sat = zeros(N,1);
    e = zeros(N,1); e_sat = zeros(N,1); u_i = zeros(N,1); u_d = zeros(N,1);
    y1 = zeros(N,1); y2 = zeros(N,1);
    delay_steps = round(L/Ts); e_prev = 0; d_prev = 0; y_prev = 0; y_prev2 = 0; u_d_prev = 0;

    for k = 2:N
        e(k) = w(k) - y(k-1);
        params.e = e(k); params.e_prev = e_prev; params.e_sat_prev = e_sat(k-1);
        params.Kp = Kp; params.Ki = Ki; params.Kd = Kd; params.Ts = Ts;
        params.u_i_prev = u_i(k-1); params.u_c_prev = u_c(max(k-1,1));
        params.u_sat_prev = u_sat(max(k-1,1)); params.u_lim = u_lim;
        params.strategy = strategy; params.N_filter = 10; params.d_prev = d_prev;
        params.y = y(max(k-1,1)); params.y_prev = y_prev; params.y_prev2 = y_prev2;
        params.w = w(k); params.K = K; params.T = T_eff; params.L = L; params.u_d_prev = u_d_prev;
        [u_c(k), u_sat(k), u_i(k), e_sat(k), u_d(k)] = PID(params);

        if k > delay_steps, u_delayed = u_sat(k-delay_steps); else, u_delayed = 0; end
        dy1 = y2(k-1); dy2 = (K*u_delayed - a1*y2(k-1) - a0*y1(k-1))/a2;
        y1(k) = y1(k-1) + Ts*dy1; y2(k) = y2(k-1) + Ts*dy2; y(k) = y1(k);
        e_prev = e(k); d_prev = u_d(k); y_prev2 = y_prev; y_prev = y(k-1); u_d_prev = u_d(k);
    end
    IAE = sum(abs(e))*Ts; ITAE = sum((1:N)'.*abs(e))*Ts;
end

%% ========== PIDF仿真 ==========
function [IAE, ITAE, y, u_sat] = simulate_pidf_osc(K, a2, a1, a0, L, Kp, Ki, Kd, Kff, Kff_d, Ts, N, w, u_lim, strategy, T_eff)
    y = zeros(N,1); u_c = zeros(N,1); u_sat = zeros(N,1);
    e = zeros(N,1); e_sat = zeros(N,1); u_i = zeros(N,1); u_d = zeros(N,1);
    y1 = zeros(N,1); y2 = zeros(N,1);
    delay_steps = round(L/Ts); e_prev = 0; d_prev = 0; y_prev = 0; y_prev2 = 0; u_d_prev = 0; w_prev = 0;

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
        [u_c(k), u_sat(k), u_i(k), e_sat(k), u_d(k)] = PIDF(params);

        if k > delay_steps, u_delayed = u_sat(k-delay_steps); else, u_delayed = 0; end
        dy1 = y2(k-1); dy2 = (K*u_delayed - a1*y2(k-1) - a0*y1(k-1))/a2;
        y1(k) = y1(k-1) + Ts*dy1; y2(k) = y2(k-1) + Ts*dy2; y(k) = y1(k);
        e_prev = e(k); d_prev = u_d(k); y_prev2 = y_prev; y_prev = y(k-1);
        u_d_prev = u_d(k); w_prev = w(k);
    end
    IAE = sum(abs(e))*Ts; ITAE = sum((1:N)'.*abs(e))*Ts;
end

%% ========== 绘制时域图 ==========
function plot_time_domain_curves(t, w, L_T, x, y_pi, u_sat_pi, strategies_pi, y_pid, u_sat_pid, strategies_pid, y_pidf, u_sat_pidf, u_lim, folders)
    num_pi = length(strategies_pi); num_pid = length(strategies_pid);
    colors_pi = lines(num_pi); colors_pid = lines(num_pid);
    show_idx = t <= 15;

    % PI
    figure('Visible','off'); hold on;
    plot(t(show_idx), w(show_idx), 'k--', 'LineWidth', 2, 'DisplayName', '设定值');
    for s = 1:num_pi, plot(t(show_idx), y_pi(show_idx,s), '-', 'Color', colors_pi(s,:), 'LineWidth', 1.5, 'DisplayName', strategies_pi{s}); end
    xlabel('时间 (s)'); ylabel('输出 y'); title(sprintf('PI 输出 (L/T=%.2f, x=%.1f)', L_T, x));
    legend('Location','best','FontSize',8); grid on; hold off;
    saveas(gcf, fullfile(folders{1}, sprintf('y_LT%.2f_x%.1f.png', L_T, x)));

    figure('Visible','off'); hold on;
    yline(u_lim,'k--','LineWidth',1,'DisplayName','u_{lim}'); yline(-u_lim,'k--','LineWidth',1,'HandleVisibility','off');
    for s = 1:num_pi, plot(t(show_idx), u_sat_pi(show_idx,s), '-', 'Color', colors_pi(s,:), 'LineWidth', 1.5, 'DisplayName', strategies_pi{s}); end
    xlabel('时间 (s)'); ylabel('u_{sat}'); title(sprintf('PI 控制器 (L/T=%.2f, x=%.1f)', L_T, x));
    legend('Location','best','FontSize',8); grid on; ylim([-1.5,1.5]); hold off;
    saveas(gcf, fullfile(folders{1}, sprintf('u_LT%.2f_x%.1f.png', L_T, x)));

    % PID
    figure('Visible','off'); hold on;
    plot(t(show_idx), w(show_idx), 'k--', 'LineWidth', 2, 'DisplayName', '设定值');
    for s = 1:num_pid, plot(t(show_idx), y_pid(show_idx,s), '-', 'Color', colors_pid(s,:), 'LineWidth', 1.5, 'DisplayName', strategies_pid{s}); end
    xlabel('时间 (s)'); ylabel('输出 y'); title(sprintf('PID 输出 (L/T=%.2f, x=%.1f)', L_T, x));
    legend('Location','best','FontSize',8); grid on; hold off;
    saveas(gcf, fullfile(folders{2}, sprintf('y_LT%.2f_x%.1f.png', L_T, x)));

    figure('Visible','off'); hold on;
    yline(u_lim,'k--','LineWidth',1,'DisplayName','u_{lim}'); yline(-u_lim,'k--','LineWidth',1,'HandleVisibility','off');
    for s = 1:num_pid, plot(t(show_idx), u_sat_pid(show_idx,s), '-', 'Color', colors_pid(s,:), 'LineWidth', 1.5, 'DisplayName', strategies_pid{s}); end
    xlabel('时间 (s)'); ylabel('u_{sat}'); title(sprintf('PID 控制器 (L/T=%.2f, x=%.1f)', L_T, x));
    legend('Location','best','FontSize',8); grid on; ylim([-1.5,1.5]); hold off;
    saveas(gcf, fullfile(folders{2}, sprintf('u_LT%.2f_x%.1f.png', L_T, x)));

    % PIDF
    figure('Visible','off'); hold on;
    plot(t(show_idx), w(show_idx), 'k--', 'LineWidth', 2, 'DisplayName', '设定值');
    for s = 1:num_pid, plot(t(show_idx), y_pidf(show_idx,s), '-', 'Color', colors_pid(s,:), 'LineWidth', 1.5, 'DisplayName', strategies_pid{s}); end
    xlabel('时间 (s)'); ylabel('输出 y'); title(sprintf('PIDF 输出 (L/T=%.2f, x=%.1f)', L_T, x));
    legend('Location','best','FontSize',8); grid on; hold off;
    saveas(gcf, fullfile(folders{3}, sprintf('y_LT%.2f_x%.1f.png', L_T, x)));

    figure('Visible','off'); hold on;
    yline(u_lim,'k--','LineWidth',1,'DisplayName','u_{lim}'); yline(-u_lim,'k--','LineWidth',1,'HandleVisibility','off');
    for s = 1:num_pid, plot(t(show_idx), u_sat_pidf(show_idx,s), '-', 'Color', colors_pid(s,:), 'LineWidth', 1.5, 'DisplayName', strategies_pid{s}); end
    xlabel('时间 (s)'); ylabel('u_{sat}'); title(sprintf('PIDF 控制器 (L/T=%.2f, x=%.1f)', L_T, x));
    legend('Location','best','FontSize',8); grid on; ylim([-1.5,1.5]); hold off;
    saveas(gcf, fullfile(folders{3}, sprintf('u_LT%.2f_x%.1f.png', L_T, x)));

    close all;
end

%% ========== 绘制汇总图 ==========
function plot_summary(results, folders)
    num_L_T = length(results.L_T_values); num_x = length(results.x_values);
    strategies_pi = results.strategies_pi; strategies_pid = results.strategies_pid;

    for i = 1:num_L_T
        for j = 1:num_x
            figure('Visible','off'); bar(squeeze(results.PI.IAE(i,j,:)));
            set(gca,'XTickLabel',strategies_pi,'XTickLabelRotation',45);
            ylabel('IAE'); title(sprintf('PI IAE (L/T=%.2f, x=%.1f)', results.L_T_values(i), results.x_values(j)));
            grid on; saveas(gcf, fullfile(folders{1}, sprintf('PI_IAE_LT%.2f_x%.1f.png', results.L_T_values(i), results.x_values(j))));

            figure('Visible','off'); bar(squeeze(results.PI.ITAE(i,j,:)));
            set(gca,'XTickLabel',strategies_pi,'XTickLabelRotation',45);
            ylabel('ITAE'); title(sprintf('PI ITAE (L/T=%.2f, x=%.1f)', results.L_T_values(i), results.x_values(j)));
            grid on; saveas(gcf, fullfile(folders{1}, sprintf('PI_ITAE_LT%.2f_x%.1f.png', results.L_T_values(i), results.x_values(j))));

            figure('Visible','off'); bar(squeeze(results.PID.IAE(i,j,:)));
            set(gca,'XTickLabel',strategies_pid,'XTickLabelRotation',45);
            ylabel('IAE'); title(sprintf('PID IAE (L/T=%.2f, x=%.1f)', results.L_T_values(i), results.x_values(j)));
            grid on; saveas(gcf, fullfile(folders{2}, sprintf('PID_IAE_LT%.2f_x%.1f.png', results.L_T_values(i), results.x_values(j))));

            figure('Visible','off'); bar(squeeze(results.PID.ITAE(i,j,:)));
            set(gca,'XTickLabel',strategies_pid,'XTickLabelRotation',45);
            ylabel('ITAE'); title(sprintf('PID ITAE (L/T=%.2f, x=%.1f)', results.L_T_values(i), results.x_values(j)));
            grid on; saveas(gcf, fullfile(folders{2}, sprintf('PID_ITAE_LT%.2f_x%.1f.png', results.L_T_values(i), results.x_values(j))));

            figure('Visible','off'); bar(squeeze(results.PIDF.IAE(i,j,:)));
            set(gca,'XTickLabel',strategies_pid,'XTickLabelRotation',45);
            ylabel('IAE'); title(sprintf('PIDF IAE (L/T=%.2f, x=%.1f)', results.L_T_values(i), results.x_values(j)));
            grid on; saveas(gcf, fullfile(folders{3}, sprintf('PIDF_IAE_LT%.2f_x%.1f.png', results.L_T_values(i), results.x_values(j))));

            figure('Visible','off'); bar(squeeze(results.PIDF.ITAE(i,j,:)));
            set(gca,'XTickLabel',strategies_pid,'XTickLabelRotation',45);
            ylabel('ITAE'); title(sprintf('PIDF ITAE (L/T=%.2f, x=%.1f)', results.L_T_values(i), results.x_values(j)));
            grid on; saveas(gcf, fullfile(folders{3}, sprintf('PIDF_ITAE_LT%.2f_x%.1f.png', results.L_T_values(i), results.x_values(j))));
        end
    end

    figure('Visible','off'); plot(results.t, results.w, 'b-', 'LineWidth', 2);
    xlabel('时间 (s)'); ylabel('设定值 w'); title('曲线跟踪设定值');
    grid on; saveas(gcf, fullfile(folders{1}, 'setpoint_curve.png'));
    close all;
end
