%% PID+前馈控制器（带多种抗积分饱和策略）
% 作者：Luciano
% 日期：2026-06-09
% 描述：整合多种抗积分饱和策略的PID+前馈控制器
%       u = u_p + u_i + u_d + u_ff
%       前馈项在饱和前加入，抗饱和作用于积分和微分项
%
% 支持的抗饱和策略：
%   'DBC_CLA' - 动态反计算 (Tt=Ti)
%   'IBC'     - 瞬时反计算
%   'CI'      - 条件积分
%   'CBC1'    - 条件反计算
%   'CBC2'    - 增量裁剪反计算
%   'DBC_STR' - 分段式动态反计算
%   'GBC'     - 广义反计算
%
% 输入参数结构体 fields（在PID基础上增加）：
%   Kff         - 前馈增益（比例前馈）
%   Kff_d       - 前馈微分增益（微分前馈）
%   w           - 当前设定值
%   w_prev      - 上一时刻设定值
%
% 其他参数同PID.m

function [u_c, u_sat, u_i, e_sat, u_d, u_ff] = PIDF(params)
    % 提取策略
    strategy = params.strategy;

    % 根据策略调用相应实现
    switch strategy
        case 'DBC_CLA'
            [u_c, u_sat, u_i, e_sat, u_d, u_ff] = pidf_dbc_cla(params);
        case 'IBC'
            [u_c, u_sat, u_i, e_sat, u_d, u_ff] = pidf_ibc(params);
        case 'CI'
            [u_c, u_sat, u_i, e_sat, u_d, u_ff] = pidf_ci(params);
        case 'CBC1'
            [u_c, u_sat, u_i, e_sat, u_d, u_ff] = pidf_cbc1(params);
        case 'CBC2'
            [u_c, u_sat, u_i, e_sat, u_d, u_ff] = pidf_cbc2(params);
        case 'DBC_STR'
            [u_c, u_sat, u_i, e_sat, u_d, u_ff] = pidf_dbc_str(params);
        case 'GBC'
            [u_c, u_sat, u_i, e_sat, u_d, u_ff] = pidf_gbc(params);
        otherwise
            error('未知的抗积分饱和策略: %s', strategy);
    end
end

%% ========== 辅助函数 ==========
function [N, d_prev, e_prev, Kff, Kff_d, w, w_prev, Ts] = get_params(params)
    if isfield(params, 'N_filter'), N = params.N_filter; else, N = 10; end
    if isfield(params, 'd_prev'), d_prev = params.d_prev; else, d_prev = 0; end
    if isfield(params, 'e_prev'), e_prev = params.e_prev; else, e_prev = 0; end
    if isfield(params, 'Kff'), Kff = params.Kff; else, Kff = 0; end
    if isfield(params, 'Kff_d'), Kff_d = params.Kff_d; else, Kff_d = 0; end
    if isfield(params, 'w'), w = params.w; else, w = 0; end
    if isfield(params, 'w_prev'), w_prev = params.w_prev; else, w_prev = 0; end
    Ts = params.Ts;
end

function u_ff = compute_feedforward(Kff, Kff_d, w, w_prev, Ts)
    % 前馈项：u_ff = Kff * w + Kff_d * dw/dt
    dw = (w - w_prev) / Ts;
    u_ff = Kff * w + Kff_d * dw;
end

function u_d = compute_derivative(Kd, e, e_prev, d_prev, N, Ts)
    u_d = Kd * (N * (e - e_prev) / Ts + (N - 1) * d_prev) / (1 + N);
end

%% ==================== DBC_CLA ====================
function [u_c, u_sat, u_i, e_sat, u_d, u_ff] = pidf_dbc_cla(params)
    e = params.e; e_sat_prev = params.e_sat_prev;
    Kp = params.Kp; Ki = params.Ki; Kd = params.Kd;
    u_i_prev = params.u_i_prev; u_lim = params.u_lim;

    [N, d_prev, e_prev, Kff, Kff_d, w, w_prev, Ts] = get_params(params);

    Tt = Kp / Ki;
    u_p = Kp * e;
    u_i = u_i_prev + Ki * Ts * e + (Ts / Tt) * e_sat_prev;
    u_d = compute_derivative(Kd, e, e_prev, d_prev, N, Ts);
    u_ff = compute_feedforward(Kff, Kff_d, w, w_prev, Ts);

    u_c = u_p + u_i + u_d + u_ff;
    u_sat = max(min(u_c, u_lim), -u_lim);
    e_sat = u_sat - u_c;
end

%% ==================== IBC ====================
function [u_c, u_sat, u_i, e_sat, u_d, u_ff] = pidf_ibc(params)
    e = params.e; e_sat_prev = params.e_sat_prev;
    Kp = params.Kp; Ki = params.Ki; Kd = params.Kd;
    u_i_prev = params.u_i_prev; u_lim = params.u_lim;

    [N, d_prev, e_prev, Kff, Kff_d, w, w_prev, Ts] = get_params(params);

    Tt = Ts;
    u_p = Kp * e;
    u_i = u_i_prev + Ki * Ts * e + (Ts / Tt) * e_sat_prev;
    u_d = compute_derivative(Kd, e, e_prev, d_prev, N, Ts);
    u_ff = compute_feedforward(Kff, Kff_d, w, w_prev, Ts);

    u_c = u_p + u_i + u_d + u_ff;
    u_sat = max(min(u_c, u_lim), -u_lim);
    e_sat = u_sat - u_c;
end

%% ==================== CI ====================
function [u_c, u_sat, u_i, e_sat, u_d, u_ff] = pidf_ci(params)
    e = params.e; e_sat_prev = params.e_sat_prev;
    Kp = params.Kp; Ki = params.Ki; Kd = params.Kd;
    u_i_prev = params.u_i_prev; u_lim = params.u_lim;

    [N, d_prev, e_prev, Kff, Kff_d, w, w_prev, Ts] = get_params(params);

    u_p = Kp * e;
    if e_sat_prev == 0
        u_i = u_i_prev + Ki * Ts * e;
    else
        u_i = u_i_prev;
    end
    u_d = compute_derivative(Kd, e, e_prev, d_prev, N, Ts);
    u_ff = compute_feedforward(Kff, Kff_d, w, w_prev, Ts);

    u_c = u_p + u_i + u_d + u_ff;
    u_sat = max(min(u_c, u_lim), -u_lim);
    e_sat = u_sat - u_c;
end

%% ==================== CBC1 ====================
function [u_c, u_sat, u_i, e_sat, u_d, u_ff] = pidf_cbc1(params)
    e = params.e; e_prev = params.e_prev; e_sat_prev = params.e_sat_prev;
    u_c_prev = params.u_c_prev; u_sat_prev = params.u_sat_prev;
    y = params.y; y_prev = params.y_prev; y_prev2 = params.y_prev2;
    Kp = params.Kp; Ki = params.Ki; Kd = params.Kd;
    u_i_prev = params.u_i_prev; u_lim = params.u_lim;

    [N, d_prev, ~, Kff, Kff_d, w, w_prev, Ts] = get_params(params);

    Ti = Kp / Ki;
    Tt = 0.03 * Ti;
    u_p = Kp * e;

    saturated = (u_c_prev ~= u_sat_prev);
    same_direction = (u_c_prev * e_prev > 0);
    output_moving = (y_prev > y_prev2 && y > y_prev2) || (y_prev < y_prev2 && y < y_prev2);
    condition = saturated && same_direction && output_moving;

    if condition
        u_i = u_i_prev + Ki * Ts * e + (Ts / Tt) * e_sat_prev;
    else
        u_i = u_i_prev + Ki * Ts * e;
    end

    u_d = compute_derivative(Kd, e, e_prev, d_prev, N, Ts);
    u_ff = compute_feedforward(Kff, Kff_d, w, w_prev, Ts);

    u_c = u_p + u_i + u_d + u_ff;
    u_sat = max(min(u_c, u_lim), -u_lim);
    e_sat = u_sat - u_c;
end

%% ==================== CBC2 ====================
function [u_c, u_sat, u_i, e_sat, u_d, u_ff] = pidf_cbc2(params)
    e = params.e; e_prev = params.e_prev; e_sat_prev = params.e_sat_prev;
    Kp = params.Kp; Ki = params.Ki; Kd = params.Kd;
    u_c_prev = params.u_c_prev; u_lim = params.u_lim;

    [N, d_prev, e_prev2, Kff, Kff_d, w, w_prev, Ts] = get_params(params);
    if isfield(params, 'e_prev2'), e_prev2 = params.e_prev2; end

    Tt = Kp / Ki;

    du_p = Kp * (e - e_prev);
    du_i = Ki * Ts * e;
    du_d = Kd * (N * ((e - e_prev) - (e_prev - e_prev2)) / Ts + (N - 1) * d_prev) / (1 + N);

    if e_sat_prev * du_i < 0
        du_clip = min(abs(e_sat_prev), abs(du_i));
        du_i = du_i - sign(du_i) * du_clip;
    end
    du_i = du_i + (Ts / Tt) * e_sat_prev;

    % 前馈增量
    u_ff = compute_feedforward(Kff, Kff_d, w, w_prev, Ts);
    if isfield(params, 'w_prev2')
        u_ff_prev = compute_feedforward(Kff, Kff_d, w_prev, params.w_prev2, Ts);
    else
        u_ff_prev = 0;
    end
    du_ff = u_ff - u_ff_prev;

    u_c = u_c_prev + du_p + du_i + du_d + du_ff;
    u_sat = max(min(u_c, u_lim), -u_lim);
    u_i = u_c - Kp * e - Kd * (e - e_prev) / Ts - u_ff;
    e_sat = u_sat - u_c;
    u_d = du_d;
end

%% ==================== DBC_STR ====================
function [u_c, u_sat, u_i, e_sat, u_d, u_ff] = pidf_dbc_str(params)
    e = params.e; e_sat_prev = params.e_sat_prev;
    Kp = params.Kp; Ki = params.Ki; Kd = params.Kd;
    u_i_prev = params.u_i_prev; u_lim = params.u_lim;
    y = params.y; K = params.K; T = params.T; L = params.L;

    [N, d_prev, e_prev, Kff, Kff_d, w, w_prev, Ts] = get_params(params);

    Ti = Kp / Ki;

    if abs(w) > 0
        R_c = u_lim * K / abs(w);
    else
        R_c = 0;
    end
    if R_c <= 2.6 && R_c >= 1
        c = -0.5 * u_lim * K / abs(w) + 1.4;
    else
        c = 0.1;
    end

    Tt_1 = 10 * Ti;
    beta = 0.59 - 0.65 * exp(-0.09 * T / L);
    Tt_new = beta * Ti;

    if abs(w) > 0 && y * w > 0 && abs(y / w) > c
        Tt = Tt_new;
    else
        Tt = Tt_1;
    end
    if Tt <= 0, Tt = Ts; end

    u_p = Kp * e;
    u_i = u_i_prev + Ki * Ts * e + (Ts / Tt) * e_sat_prev;
    u_d = compute_derivative(Kd, e, e_prev, d_prev, N, Ts);
    u_ff = compute_feedforward(Kff, Kff_d, w, w_prev, Ts);

    u_c = u_p + u_i + u_d + u_ff;
    u_sat = max(min(u_c, u_lim), -u_lim);
    e_sat = u_sat - u_c;
end

%% ==================== GBC ====================
function [u_c, u_sat, u_i, e_sat, u_d, u_ff] = pidf_gbc(params)
    e = params.e; e_sat_prev = params.e_sat_prev;
    Kp = params.Kp; Ki = params.Ki; Kd = params.Kd;
    u_i_prev = params.u_i_prev; u_lim = params.u_lim;

    [N, d_prev, e_prev, Kff, Kff_d, w, w_prev, Ts] = get_params(params);

    if isfield(params, 'u_d_prev')
        u_d_prev = params.u_d_prev;
    else
        u_d_prev = 0;
    end

    c_inf = Kp + Kd * N;
    alpha = 1 / c_inf;

    u_p = Kp * e;
    u_i = u_i_prev + Ki * Ts * e + alpha * Ts * e_sat_prev;

    a = N / (N + 1);
    u_d = a * u_d_prev + Kd * N * (e - e_prev) / (Ts * (N + 1));
    u_d = u_d + alpha * e_sat_prev;

    u_ff = compute_feedforward(Kff, Kff_d, w, w_prev, Ts);

    u_c = u_p + u_i + u_d + u_ff;
    u_sat = max(min(u_c, u_lim), -u_lim);
    e_sat = u_sat - u_c;
end
