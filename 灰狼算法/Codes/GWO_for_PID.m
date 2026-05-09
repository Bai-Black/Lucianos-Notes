%种群数量
N = 20;
%上下限
Kp_max = 200;
Kp_min = 0;
Ki_max = 200;
Ki_min = 0;
Kd_max = 100;
Kd_min = 0;
%迭代次数
interate_num = 30;

function value = PidTest(K)
    % K = [Kp Ki Kd]

    % 传参数到 Simulink
    assignin('base','Kp',K(1));
    assignin('base','Ki',K(2));
    assignin('base','Kd',K(3));

    % 运行仿真
    simOut = sim('pid_test','StopTime','30');

    value = simOut.get('ITAE').Data(end)
end

K = rand(N, 3) .* [Kp_max-Kp_min, Ki_max-Ki_min, Kd_max-Kd_min] + [Kp_min, Ki_min, Kd_min]

alpha_value = inf;
beta_value = inf;
delta_value = inf;

interate = 0;
value_history = [];
alpha = [];
beta = [];
delta = [];
while interate < interate_num
    for i = 1:N
        value = PidTest(K(i, :));
        if value < alpha_value
            delta = beta;
            delta_value = beta_value;
            beta = alpha;
            beta_value = alpha_value;

            alpha = K(i, :);
            alpha_value = value;
        elseif value < beta_value
            delta = beta;
            delta_value = beta_value;

            beta = K(i, :);
            beta_value = value;
        elseif value < delta_value
            delta = K(i, :);
            delta_value = value;
        end
    end

    a = 2 - 2 / interate_num * interate;

    A1 = 2 * a * rand(N, 3) - a;
    C1 = 2 * rand(N, 3);
    D_alpha = abs(alpha .* C1 - K);
    K1 = alpha - A1 .* D_alpha;

    A2 = 2 * a * rand(N, 3) - a;
    C2 = 2 * rand(N, 3);
    D_beta = abs(beta .* C2 - K);
    K2 = beta - A2 .* D_beta;

    A3 = 2 * a * rand(N, 3) - a;
    C3 = 2 * rand(N, 3);
    D_delta = abs(delta .* C3 - K);
    K3 = delta - A3 .* D_delta;

    K = (K1 + K2 + K3) / 3;
    K(:, 1) = clip(K(:, 1), Kp_min, Kp_max);
    K(:, 2) = clip(K(:, 2), Ki_min, Ki_max);
    K(:, 3) = clip(K(:, 3), Kd_min, Kd_max);
    interate = interate + 1;
    value_history(end+1) = alpha_value
end

alpha
plot(value_history)