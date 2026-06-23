# PID抗积分饱和仿真代码

## 文件结构

```
simulation/
├── 控制器文件
│   ├── PI.m                        % PI控制器（6种策略）
│   ├── PID.m                       % PID控制器（7种策略）
│   └── PIDF.m                      % PID+前馈控制器（7种策略）
│
├── 情景一：简化速度环
│   ├── run_scenario1_PI.m          % 阶跃响应 → 1_1
│   ├── run_scenario1_PID.m         % 阶跃响应 → 1_2
│   └── run_scenario1_curve.m       % 曲线跟踪 → 1_3, 1_4, 1_5
│
├── 情景二：真实速度环
│   ├── run_scenario2_PI.m          % 阶跃响应 → 2_1
│   ├── run_scenario2_PID.m         % 阶跃响应 → 2_2
│   └── run_scenario2_curve.m       % 曲线跟踪 → 2_3, 2_4, 2_5
│
├── 情景三：平衡pitch环
│   ├── run_scenario3_PI.m          % 阶跃响应 → 3_1 (旧)
│   ├── run_scenario3_PID.m         % 阶跃响应 → 3_2 (旧)
│   ├── run_scenario3_curve.m       % 曲线跟踪 → 3_3, 3_4, 3_5 (旧)
│   ├── run_scenario3_dist_small.m  % 抗持续小幅扰动 → 3_1, 3_3, 3_5
│   └── run_scenario3_dist_large.m  % 抗一次性大幅扰动 → 3_2, 3_4, 3_6
│
├── 输出文件夹
│   ├── 1_1 ~ 1_5                   % 情景一结果
│   ├── 2_1 ~ 2_5                   % 情景二结果
│   └── 3_1 ~ 3_6                   % 情景三结果
│
└── README.md
```

## 系统模型

### 情景一：简化速度环（一阶FOPDT）
$$
G_1(s) = \frac{1}{0.03s+1} e^{-Ls}, \quad T = 0.03
$$

### 情景二：真实速度环（二阶过阻尼）
$$
G_2(s) = \frac{1}{(0.02s+1)(0.05s+1)} e^{-Ls}, \quad T_{eq} = 0.05
$$

### 情景三：平衡pitch环（二阶欠阻尼）
$$
G_3(s) = \frac{1}{0.04s^2+0.1s+1} e^{-Ls}, \quad \omega_n = 5, \quad \zeta = 0.25
$$

## 公共参数

| 参数 | 取值 |
|------|------|
| L/T_eq | 0.02, 0.1, 0.3, 0.5 |
| x | 0.2, 0.5, 0.8 |
| u_lim | ±1 |

## 控制器文件

### PI.m

PI控制器，支持6种抗积分饱和策略。

| 策略 | 编码 | 说明 |
|------|------|------|
| 动态反计算 | `DBC_CLA` | Tt=Ti |
| 瞬时反计算 | `IBC` | Tt=Ts |
| 条件积分 | `CI` | 饱和时停止积分 |
| 条件反计算 | `CBC1` | 大延迟适用 |
| 增量裁剪反计算 | `CBC2` | 两级抗饱和 |
| 分段式动态反计算 | `DBC_STR` | 先猛冲再收敛 |

```matlab
[u_c, u_sat, u_i, e_sat] = PI(params);
```

### PID.m

PID控制器，支持7种策略（增加GBC）。

| 策略 | 编码 | 说明 |
|------|------|------|
| 广义反计算 | `GBC` | 对所有内部状态施加抗饱和 |

```matlab
params.Kd = 0.01;
params.N_filter = 10;
[u_c, u_sat, u_i, e_sat, u_d] = PID(params);
```

### PIDF.m

PID+前馈控制器，支持7种策略。

```matlab
params.Kff = 0.5;       % 比例前馈
params.Kff_d = 0.01;    % 微分前馈
[u_c, u_sat, u_i, e_sat, u_d, u_ff] = PIDF(params);
```

## 仿真脚本

### 情景一：简化速度环

| 脚本 | 控制器 | 输出 |
|------|--------|------|
| `run_scenario1_PI.m` | PI | `1_1` |
| `run_scenario1_PID.m` | PID | `1_2` |
| `run_scenario1_curve.m` | PI/PID/PIDF | `1_3`/`1_4`/`1_5` |

### 情景二：真实速度环

| 脚本 | 控制器 | 输出 |
|------|--------|------|
| `run_scenario2_PI.m` | PI | `2_1` |
| `run_scenario2_PID.m` | PID | `2_2` |
| `run_scenario2_curve.m` | PI/PID/PIDF | `2_3`/`2_4`/`2_5` |

### 情景三：平衡pitch环

| 脚本 | 测试内容 | 输出 |
|------|----------|------|
| `run_scenario3_dist_small.m` | 抗持续小幅扰动 | `3_1`(PI), `3_3`(PID), `3_5`(PIDF) |
| `run_scenario3_dist_large.m` | 抗一次性大幅扰动 | `3_2`(PI), `3_4`(PID), `3_6`(PIDF) |

## 运行方式

```matlab
cd simulation

%% ===== 情景一 =====
run_scenario1_PI           % 阶跃响应 PI
run_scenario1_PID          % 阶跃响应 PID
run_scenario1_curve        % 曲线跟踪

%% ===== 情景二 =====
run_scenario2_PI           % 阶跃响应 PI
run_scenario2_PID          % 阶跃响应 PID
run_scenario2_curve        % 曲线跟踪

%% ===== 情景三 =====
run_scenario3_dist_small   % 抗持续小幅扰动
run_scenario3_dist_large   % 抗一次性大幅扰动
```

## PID参数整定（λ方法）

**PI控制器：**
```
Kp = T_eq / (K × (λ + L))
Ti = T_eq
Ki = Kp / Ti
```

**PID控制器：**
```
Kp = (T_eq + 0.5L) / (K × (λ + 0.5L))
Ti = T_eq + 0.5L
Td = T_eq × L / (2T_eq + L)
Kd = Kp × Td
```

其中 λ = x × T_eq。

## 性能指标

**IAE（绝对误差积分）：**
$$
IAE = \sum_{k=1}^{N} |e(k)| \cdot T_s
$$

**ITAE（时间加权绝对误差积分）：**
$$
ITAE = \sum_{k=1}^{N} k \cdot |e(k)| \cdot T_s
$$

## 输出图表说明

### 阶跃响应（1_1, 1_2, 2_1, 2_2）

- `IAE_norm_LT*.png` - IAE归一化图（R_S为横坐标，DBC_CLA为基准）
- `ITAE_norm_LT*.png` - ITAE归一化图
- `y_RS*_LT*.png` - 系统输出时域图
- `u_sat_RS*_LT*.png` - 控制器行为时域图

### 曲线跟踪（1_3~1_5, 2_3~2_5）

- `y_LT*.png` - 系统输出时域图
- `u_LT*.png` - 控制器行为时域图
- `*_IAE_LT*.png` - IAE柱状图
- `*_ITAE_LT*.png` - ITAE柱状图
- `setpoint_curve.png` - 设定值曲线

### 抗扰动测试（3_1~3_6）

- `*_scores_LT*.png` - IAE/ITAE柱状图
- `*_time_LT*.png` - 输出-时间图 + 控制器行为-时间图

## 参考文献

- Åström, K. J., & Hägglund, T. (2006). Advanced PID control.
- 原始研究文档：PID抗积分饱和.md
