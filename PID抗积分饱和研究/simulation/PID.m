%% PID控制器（带多种抗积分饱和策略）
% 作者：Luciano
% 日期：2026-06-09
% 描述：整合多种抗积分饱和策略的PID控制器
%       通过strategy选择抗饱和方法
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
% 输入参数结构体 fields：
%   e           - 当前误差
%   e_prev      - 上一时刻误差
%   e_sat_prev  - 上一时刻的饱和误差
%   Kp          - 比例增益
%   Ki          - 积分系数
%   Kd          - 微分系数
%   Ts          - 采样时间
%   u_i_prev    - 上一时刻的积分项
%   u_c_prev    - 上一时刻的控制器输出
%   u_sat_prev  - 上一时刻的饱和输出
%   u_lim       - 饱和限制（±u_lim）
%   strategy    - 抗积分饱和策略名称
%
%   % CBC1专用参数
%   y           - 当前系统输出
%   y_prev      - 上一时刻系统输出
%   y_prev2     - 上上时刻系统输出
%
%   % DBC_STR专用参数
%   w           - 当前设定值
%   K           - 系统增益
%   T           - 时间常数
%   L           - 纯延迟时间
%
%   % 微分滤波参数
%   d_prev      - 上一时刻微分项
%   N_filter    - 微分滤波系数（默认10）
%
%   % GBC专用参数
%   u_d_prev    - 上一时刻微分滤波器状态
%
% 输出：
%   u_c         - 控制器输出（未饱和）
%   u_sat       - 控制器输出（饱和后）
%   u_i         - 当前积分项
%   e_sat       - 当前饱和误差
%   u_d         - 当前微分项

function [u_c, u_sat, u_i, e_sat, u_d] = PID(params)
    % 提取通用参数
    strategy = params.strategy;

    % 根据策略调用相应实现
    switch strategy
        case 'DBC_CLA'
            [u_c, u_sat, u_i, e_sat, u_d] = pid_dbc_cla(params);
        case 'IBC'
            [u_c, u_sat, u_i, e_sat, u_d] = pid_ibc(params);
        case 'CI'
            [u_c, u_sat, u_i, e_sat, u_d] = pid_ci(params);
        case 'CBC1'
            [u_c, u_sat, u_i, e_sat, u_d] = pid_cbc1(params);
        case 'CBC2'
            [u_c, u_sat, u_i, e_sat, u_d] = pid_cbc2(params);
        case 'DBC_STR'
            [u_c, u_sat, u_i, e_sat, u_d] = pid_dbc_str(params);
        case 'GBC'
            [u_c, u_sat, u_i, e_sat, u_d] = pid_gbc(params);
        otherwise
            error('未知的抗积分饱和策略: %s', strategy);
    end
end

%% ========== 辅助函数：获取微分滤波参数 ==========
function [N, d_prev, e_prev] = get_filter_params(params)
    if isfield(params, 'N_filter')
        N = params.N_filter;
    else
        N = 10;
    end
    if isfield(params, 'd_prev')
        d_prev = params.d_prev;
    else
        d_prev = 0;
    end
    if isfield(params, 'e_prev')
        e_prev = params.e_prev;
    else
        e_prev = 0;
    end
end

%% ========== 辅助函数：计算带滤波的微分项 ==========
function u_d = compute_derivative(Kd, e, e_prev, d_prev, N, Ts)
    % 带一阶滤波的微分
    % u_d = Kd * [N*(e-e_prev)/Ts + (N-1)*d_prev/Ts] / (1+N)
    u_d = Kd * (N * (e - e_prev) / Ts + (N - 1) * d_prev) / (1 + N);
end

%% ==================== DBC_CLA：动态反计算 PID ====================
function [u_c, u_sat, u_i, e_sat, u_d] = pid_dbc_cla(params)
    e = params.e; e_sat_prev = params.e_sat_prev;
    Kp = params.Kp; Ki = params.Ki; Kd = params.Kd;
    Ts = params.Ts; u_i_prev = params.u_i_prev; u_lim = params.u_lim;

    [N, d_prev, e_prev] = get_filter_params(params);

    % 跟踪时间常数
    Tt = Kp / Ki;

    % 比例项
    u_p = Kp * e;

    % 积分项（动态反计算）
    u_i = u_i_prev + Ki * Ts * e + (Ts / Tt) * e_sat_prev;

    % 微分项
    u_d = compute_derivative(Kd, e, e_prev, d_prev, N, Ts);

    % 控制器输出
    u_c = u_p + u_i + u_d;

    % 饱和
    u_sat = max(min(u_c, u_lim), -u_lim);

    % 饱和误差
    e_sat = u_sat - u_c;
end

%% ==================== IBC：瞬时反计算 PID ====================
function [u_c, u_sat, u_i, e_sat, u_d] = pid_ibc(params)
    e = params.e; e_sat_prev = params.e_sat_prev;
    Kp = params.Kp; Ki = params.Ki; Kd = params.Kd;
    Ts = params.Ts; u_i_prev = params.u_i_prev; u_lim = params.u_lim;

    [N, d_prev, e_prev] = get_filter_params(params);

    % 跟踪时间常数（瞬时反计算）
    Tt = Ts;

    u_p = Kp * e;
    u_i = u_i_prev + Ki * Ts * e + (Ts / Tt) * e_sat_prev;
    u_d = compute_derivative(Kd, e, e_prev, d_prev, N, Ts);

    u_c = u_p + u_i + u_d;
    u_sat = max(min(u_c, u_lim), -u_lim);
    e_sat = u_sat - u_c;
end

%% ==================== CI：条件积分 PID ====================
function [u_c, u_sat, u_i, e_sat, u_d] = pid_ci(params)
    e = params.e; e_sat_prev = params.e_sat_prev;
    Kp = params.Kp; Ki = params.Ki; Kd = params.Kd;
    Ts = params.Ts; u_i_prev = params.u_i_prev; u_lim = params.u_lim;

    [N, d_prev, e_prev] = get_filter_params(params);

    u_p = Kp * e;

    % 条件积分
    if e_sat_prev == 0
        u_i = u_i_prev + Ki * Ts * e;
    else
        u_i = u_i_prev;
    end

    u_d = compute_derivative(Kd, e, e_prev, d_prev, N, Ts);

    u_c = u_p + u_i + u_d;
    u_sat = max(min(u_c, u_lim), -u_lim);
    e_sat = u_sat - u_c;
end

%% ==================== CBC1：条件反计算 PID ====================
function [u_c, u_sat, u_i, e_sat, u_d] = pid_cbc1(params)
    e = params.e; e_prev = params.e_prev; e_sat_prev = params.e_sat_prev;
    u_c_prev = params.u_c_prev; u_sat_prev = params.u_sat_prev;
    y = params.y; y_prev = params.y_prev; y_prev2 = params.y_prev2;
    Kp = params.Kp; Ki = params.Ki; Kd = params.Kd;
    Ts = params.Ts; u_i_prev = params.u_i_prev; u_lim = params.u_lim;

    [N, d_prev, ~] = get_filter_params(params);

    Ti = Kp / Ki;
    Tt = 0.03 * Ti;

    u_p = Kp * e;

    % 判断反计算是否启用
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

    u_c = u_p + u_i + u_d;
    u_sat = max(min(u_c, u_lim), -u_lim);
    e_sat = u_sat - u_c;
end

%% ==================== CBC2：增量裁剪反计算 PID ====================
function [u_c, u_sat, u_i, e_sat, u_d] = pid_cbc2(params)
    e = params.e; e_prev = params.e_prev; e_sat_prev = params.e_sat_prev;
    Kp = params.Kp; Ki = params.Ki; Kd = params.Kd;
    Ts = params.Ts; u_c_prev = params.u_c_prev; u_lim = params.u_lim;

    [N, d_prev, e_prev2] = get_filter_params(params);
    if isfield(params, 'e_prev2')
        e_prev2 = params.e_prev2;
    end

    Tt = Kp / Ki;

    % 增量计算
    du_p = Kp * (e - e_prev);
    du_i = Ki * Ts * e;
    du_d = Kd * (N * ((e - e_prev) - (e_prev - e_prev2)) / Ts + (N - 1) * d_prev) / (1 + N);

    % 增量裁剪
    if e_sat_prev * du_i < 0
        du_clip = min(abs(e_sat_prev), abs(du_i));
        du_i = du_i - sign(du_i) * du_clip;
    end

    % 反计算
    du_i = du_i + (Ts / Tt) * e_sat_prev;

    % 控制器输出
    u_c = u_c_prev + du_p + du_i + du_d;
    u_sat = max(min(u_c, u_lim), -u_lim);
    u_i = u_c - Kp * e - Kd * (e - e_prev) / Ts;
    e_sat = u_sat - u_c;
    u_d = du_d;
end

%% ==================== DBC_STR：分段式动态反计算 PID ====================
function [u_c, u_sat, u_i, e_sat, u_d] = pid_dbc_str(params)
    e = params.e; e_sat_prev = params.e_sat_prev;
    Kp = params.Kp; Ki = params.Ki; Kd = params.Kd;
    Ts = params.Ts; u_i_prev = params.u_i_prev; u_lim = params.u_lim;
    y = params.y; w = params.w; K = params.K; T = params.T; L = params.L;

    [N, d_prev, e_prev] = get_filter_params(params);

    Ti = Kp / Ki;

    % 计算切换阈值
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
    if Tt <= 0
        Tt = Ts;
    end

    u_p = Kp * e;
    u_i = u_i_prev + Ki * Ts * e + (Ts / Tt) * e_sat_prev;
    u_d = compute_derivative(Kd, e, e_prev, d_prev, N, Ts);

    u_c = u_p + u_i + u_d;
    u_sat = max(min(u_c, u_lim), -u_lim);
    e_sat = u_sat - u_c;
end

%% ==================== GBC：广义反计算 PID ====================
% 广义反计算原理：
% 将控制器分解为 C(s) = c_∞ + C̄(s)
%   c_∞ = lim(s→∞) C(s) 为高频增益（直通项）
%   C̄(s) 为严格真传递函数（包含积分和微分动态）
%
% 对于带滤波微分的PID：
%   c_∞ = Kp + Kd*N  （高频时微分滤波器增益为N）
%   抗饱和增益 α = 1/c_∞
%
% 与经典反计算不同，GBC对所有内部状态（积分+微分滤波器）
% 施加统一的抗饱和反馈
function [u_c, u_sat, u_i, e_sat, u_d] = pid_gbc(params)
    e = params.e; e_sat_prev = params.e_sat_prev;
    Kp = params.Kp; Ki = params.Ki; Kd = params.Kd;
    Ts = params.Ts; u_i_prev = params.u_i_prev; u_lim = params.u_lim;

    [N, d_prev, e_prev] = get_filter_params(params);

    % 微分滤波器状态
    if isfield(params, 'u_d_prev')
        u_d_prev = params.u_d_prev;
    else
        u_d_prev = 0;
    end

    % ========== 广义反计算核心 ==========
    % 高频增益 c_∞
    % 对于PID+滤波微分：C(s) = Kp + Kp/(Ti*s) + Kd*s/(1+s/N)
    % 高频时：积分项→0，微分滤波器→N
    % 所以 c_∞ = Kp + Kd*N（如果Kd已包含Kp*Td则不需要额外加Kp）
    c_inf = Kp + Kd * N;

    % 抗饱和增益
    alpha = 1 / c_inf;

    % ========== 比例项 ==========
    u_p = Kp * e;

    % ========== 积分项（带GBC抗饱和） ==========
    % 积分器动态 + 抗饱和反馈
    u_i = u_i_prev + Ki * Ts * e + alpha * Ts * e_sat_prev;

    % ========== 微分项（带GBC抗饱和） ==========
    % 微分滤波器：一阶低通滤波
    % 离散化：u_d(k) = a*u_d(k-1) + b*(e(k)-e(k-1))
    % 其中 a = (N*Td - Ts)/(N*Td + Ts), b = Kd*N/(N*Td + Ts)
    % 简化形式（使用一阶后向差分）：
    a = N / (N + 1);  % 滤波系数
    u_d = a * u_d_prev + Kd * N * (e - e_prev) / (Ts * (N + 1));

    % 微分项的抗饱和反馈
    u_d = u_d + alpha * e_sat_prev;

    % ========== 控制器输出 ==========
    u_c = u_p + u_i + u_d;

    % ========== 饱和 ==========
    u_sat = max(min(u_c, u_lim), -u_lim);

    % ========== 饱和误差 ==========
    e_sat = u_sat - u_c;
end
