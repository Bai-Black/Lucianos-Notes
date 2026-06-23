%% PI控制器（带多种抗积分饱和策略）
% 作者：Luciano
% 日期：2026-06-09
% 描述：整合多种抗积分饱和策略的PI控制器
%       通过strategy参数选择不同的抗饱和方法
%
% 支持的策略：
%   'DBC_CLA' - 动态反计算 (Tt=Ti)
%   'IBC'     - 瞬时反计算
%   'CI'      - 条件积分
%   'CBC1'    - 条件反计算
%   'CBC2'    - 增量裁剪反计算
%   'DBC_STR' - 分段式动态反计算
%
% 输入参数结构体 fields：
%   e         - 当前误差
%   e_prev    - 上一时刻误差（CBC1, CBC2需要）
%   e_sat_prev - 上一时刻的饱和误差
%   Kp        - 比例增益
%   Ki        - 积分系数
%   Ts        - 采样时间
%   u_i_prev  - 上一时刻的积分项
%   u_c_prev  - 上一时刻的控制器输出（CBC1, CBC2需要）
%   u_sat_prev - 上一时刻的饱和输出（CBC1需要）
%   u_lim     - 饱和限制（±u_lim）
%   strategy  - 抗积分饱和策略名称
%
%   % CBC1专用参数
%   y         - 当前系统输出
%   y_prev    - 上一时刻系统输出
%   y_prev2   - 上上时刻系统输出
%
%   % DBC_STR专用参数
%   w         - 当前设定值
%   K         - 系统增益
%   T         - 时间常数
%   L         - 纯延迟时间
%
% 输出：
%   u_c       - 控制器输出（未饱和）
%   u_sat     - 控制器输出（饱和后）
%   u_i       - 当前积分项
%   e_sat     - 当前饱和误差

function [u_c, u_sat, u_i, e_sat] = PI(params)
    % 提取通用参数
    e = params.e;
    e_sat_prev = params.e_sat_prev;
    Kp = params.Kp;
    Ki = params.Ki;
    Ts = params.Ts;
    u_i_prev = params.u_i_prev;
    u_lim = params.u_lim;
    strategy = params.strategy;

    % 根据策略调用相应实现
    switch strategy
        case 'DBC_CLA'
            [u_c, u_sat, u_i, e_sat] = pi_dbc_cla(e, e_sat_prev, Kp, Ki, Ts, u_i_prev, u_lim);
        case 'IBC'
            [u_c, u_sat, u_i, e_sat] = pi_ibc(e, e_sat_prev, Kp, Ki, Ts, u_i_prev, u_lim);
        case 'CI'
            [u_c, u_sat, u_i, e_sat] = pi_ci(e, e_sat_prev, Kp, Ki, Ts, u_i_prev, u_lim);
        case 'CBC1'
            [u_c, u_sat, u_i, e_sat] = pi_cbc1(params);
        case 'CBC2'
            [u_c, u_sat, u_i, e_sat] = pi_cbc2(params);
        case 'DBC_STR'
            [u_c, u_sat, u_i, e_sat] = pi_dbc_str(params);
        otherwise
            error('未知的抗积分饱和策略: %s', strategy);
    end

end

%% ========== DBC_CLA：动态反计算 ==========
% 经典动态反计算策略，跟踪时间常数 Tt = Ti
function [u_c, u_sat, u_i, e_sat] = pi_dbc_cla(e, e_sat_prev, Kp, Ki, Ts, u_i_prev, u_lim)
    % 跟踪时间常数
    Tt = Kp / Ki;

    % 比例项
    u_p = Kp * e;

    % 积分项（动态反计算）
    u_i = u_i_prev + Ki * Ts * e + (Ts / Tt) * e_sat_prev;

    % 控制器输出
    u_c = u_p + u_i;

    % 饱和
    u_sat = max(min(u_c, u_lim), -u_lim);

    % 饱和误差
    e_sat = u_sat - u_c;
end

%% ========== IBC：瞬时反计算 ==========
% 控制信号钳位，Tt = Ts，快速退出饱和
function [u_c, u_sat, u_i, e_sat] = pi_ibc(e, e_sat_prev, Kp, Ki, Ts, u_i_prev, u_lim)
    % 跟踪时间常数（瞬时反计算）
    Tt = Ts;

    % 比例项
    u_p = Kp * e;

    % 积分项（瞬时反计算）
    u_i = u_i_prev + Ki * Ts * e + (Ts / Tt) * e_sat_prev;

    % 控制器输出
    u_c = u_p + u_i;

    % 饱和
    u_sat = max(min(u_c, u_lim), -u_lim);

    % 饱和误差
    e_sat = u_sat - u_c;
end

%% ========== CI：条件积分 ==========
% 当控制器饱和时停止积分
function [u_c, u_sat, u_i, e_sat] = pi_ci(e, e_sat_prev, Kp, Ki, Ts, u_i_prev, u_lim)
    % 比例项
    u_p = Kp * e;

    % 积分项（条件积分）
    if e_sat_prev == 0
        % 未饱和：正常积分
        u_i = u_i_prev + Ki * Ts * e;
    else
        % 已饱和：暂停积分
        u_i = u_i_prev;
    end

    % 控制器输出
    u_c = u_p + u_i;

    % 饱和
    u_sat = max(min(u_c, u_lim), -u_lim);

    % 饱和误差
    e_sat = u_sat - u_c;
end

%% ========== CBC1：条件反计算 ==========
% 结合条件积分与动态反计算，适用于大延迟系统
function [u_c, u_sat, u_i, e_sat] = pi_cbc1(params)
    % 提取参数
    e = params.e;
    e_prev = params.e_prev;
    e_sat_prev = params.e_sat_prev;
    u_c_prev = params.u_c_prev;
    u_sat_prev = params.u_sat_prev;
    y = params.y;
    y_prev = params.y_prev;
    y_prev2 = params.y_prev2;
    Kp = params.Kp;
    Ki = params.Ki;
    Ts = params.Ts;
    u_i_prev = params.u_i_prev;
    u_lim = params.u_lim;

    % 跟踪时间常数
    Ti = Kp / Ki;
    Tt = 0.03 * Ti;

    % 比例项
    u_p = Kp * e;

    % 判断反计算是否启用的三个条件
    saturated = (u_c_prev ~= u_sat_prev);
    same_direction = (u_c_prev * e_prev > 0);
    output_moving = (y_prev > y_prev2 && y > y_prev2) || ...
                    (y_prev < y_prev2 && y < y_prev2);
    condition = saturated && same_direction && output_moving;

    % 积分项
    if condition
        u_i = u_i_prev + Ki * Ts * e + (Ts / Tt) * e_sat_prev;
    else
        u_i = u_i_prev + Ki * Ts * e;
    end

    % 控制器输出
    u_c = u_p + u_i;

    % 饱和
    u_sat = max(min(u_c, u_lim), -u_lim);

    % 饱和误差
    e_sat = u_sat - u_c;
end

%% ========== CBC2：增量裁剪反计算 ==========
% 两级抗饱和：先裁剪增量再反计算
function [u_c, u_sat, u_i, e_sat] = pi_cbc2(params)
    % 提取参数
    e = params.e;
    e_prev = params.e_prev;
    e_sat_prev = params.e_sat_prev;
    Kp = params.Kp;
    Ki = params.Ki;
    Ts = params.Ts;
    u_c_prev = params.u_c_prev;
    u_lim = params.u_lim;

    % 跟踪时间常数
    Tt = Kp / Ki;

    % 增量PI计算
    du_p = Kp * (e - e_prev);
    du_i = Ki * Ts * e;

    % 第一阶段：增量裁剪
    if e_sat_prev * du_i < 0
        du_clip = min(abs(e_sat_prev), abs(du_i));
        du_i = du_i - sign(du_i) * du_clip;
    end

    % 第二阶段：反计算
    du_i = du_i + (Ts / Tt) * e_sat_prev;

    % 控制器输出
    u_c = u_c_prev + du_p + du_i;

    % 饱和
    u_sat = max(min(u_c, u_lim), -u_lim);

    % 计算积分项（用于返回）
    u_i = u_c - Kp * e;

    % 饱和误差
    e_sat = u_sat - u_c;
end

%% ========== DBC_STR：分段式动态反计算 ==========
% 两阶段策略："先猛冲、再收敛"
function [u_c, u_sat, u_i, e_sat] = pi_dbc_str(params)
    % 提取参数
    e = params.e;
    e_sat_prev = params.e_sat_prev;
    Kp = params.Kp;
    Ki = params.Ki;
    Ts = params.Ts;
    u_i_prev = params.u_i_prev;
    u_lim = params.u_lim;
    y = params.y;
    w = params.w;
    K = params.K;
    T = params.T;
    L = params.L;

    % 积分时间
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

    % 计算Tt
    Tt_1 = 10 * Ti;
    beta = 0.59 - 0.65 * exp(-0.09 * T / L);
    Tt_new = beta * Ti;

    % 判断当前阶段
    if abs(w) > 0 && y * w > 0 && abs(y / w) > c
        Tt = Tt_new;
    else
        Tt = Tt_1;
    end

    % 确保Tt为正
    if Tt <= 0
        Tt = Ts;
    end

    % 比例项
    u_p = Kp * e;

    % 积分项（动态反计算）
    u_i = u_i_prev + Ki * Ts * e + (Ts / Tt) * e_sat_prev;

    % 控制器输出
    u_c = u_p + u_i;

    % 饱和
    u_sat = max(min(u_c, u_lim), -u_lim);

    % 饱和误差
    e_sat = u_sat - u_c;
end
