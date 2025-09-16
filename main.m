%% 系统参数设置
% UGV数量和UAV数量
N_ugv = 4;
N_uav = 4;

% 仿真时间参数
dt = 0.001;  % 减小时间步长
T_sim = 20;  % 减少仿真时间
t_span = 0:dt:T_sim;
N_steps = length(t_span);

% 目标位置
p_d = [10; 10];  % UGV队形几何中心目标位置  
p_d_uav = [10; 10; 2];  % UAV队形几何中心目标位置

%% 队形向量定义
% UGV期望队形向量 (2D)
w = [1, -1, -1, 1; 1, 1, -1, -1];  % 2x4矩阵：UGV1=[1,1], UGV2=[-1,1], UGV3=[-1,-1], UGV4=[1,-1]
% UAV期望队形向量 (3D)  
r = [1, -1, -1, 1; 1, 1, -1, -1; 0, 0, 0, 0];  % 3x4矩阵：对应UGV位置上方2m

%% 通信拓扑矩阵（暂时不用）
% UGV通信拓扑 A (环形拓扑: 1-2, 2-3, 3-4, 4-1)
A = [0, 1, 0, 1;
     1, 0, 1, 0;
     0, 1, 0, 1;
     1, 0, 1, 0];

% UAV通信拓扑 B (同UGV)
B = A;

% UGV-UAV通信拓扑 C (对角矩阵，相同编号之间通信)
C = eye(4);

%% 状态矩阵和控制矩阵定义
% UGV状态矩阵 D_x (4x4, 包含位置和速度)
D_x = cell(N_ugv, 1);
E_x = cell(N_ugv, 1);

for i = 1:N_ugv
    % 双积分器模型 + 轻微阻尼
    damping = 0.05 + 0.01*i;  % 不同的阻尼系数
    D_x{i} = [0, 0, 1, 0;
              0, 0, 0, 1;
              0, 0, -damping, 0;
              0, 0, 0, -damping];
    E_x{i} = [0, 0; 0, 0; 1, 0; 0, 1];
end

% UAV状态矩阵 D_y (6x6, 包含xyz位置和速度)
D_y = cell(N_uav, 1);
E_y = cell(N_uav, 1);

for i = 1:N_uav
    % 三维双积分器模型 + 轻微阻尼
    damping = 0.08 + 0.02*i;
    D_y{i} = [0, 0, 0, 1, 0, 0;
              0, 0, 0, 0, 1, 0;
              0, 0, 0, 0, 0, 1;
              0, 0, 0, -damping, 0, 0;
              0, 0, 0, 0, -damping, 0;
              0, 0, 0, 0, 0, -damping];
    E_y{i} = [0, 0, 0; 0, 0, 0; 0, 0, 0; 1, 0, 0; 0, 1, 0; 0, 0, 1];
end

%% 初始状态设置
% UGV初始状态 [px, py, vx, vy]
x_ugv = zeros(4, N_ugv, N_steps);
x_ugv(:, :, 1) = [0, 3, 6, 3;      % px初始位置
                  0, 0, 3, 6;      % py初始位置
                  0.1, -0.1, 0.1, -0.1;  % vx初始速度（小值）
                  0.1, 0.1, -0.1, -0.1]; % vy初始速度（小值）

% UAV初始状态 [px, py, pz, vx, vy, vz]
y_uav = zeros(6, N_uav, N_steps);
y_uav(:, :, 1) = [1, 4, 7, 4;      % px初始位置
                  1, 1, 4, 7;      % py初始位置
                  3, 4, 5, 6;      % pz初始位置
                  0.1, -0.1, 0.1, -0.1;  % vx初始速度
                  0.1, 0.1, -0.1, -0.1;  % vy初始速度
                  0, 0, 0, 0];     % vz初始速度

%% 自适应参数初始化
% 不同智能体设置不同的初值和自适应率
d_ugv = zeros(N_ugv, N_steps);
d_uav = zeros(N_uav, N_steps);

% UGV自适应增益初值（降低初值）
d_ugv(1, 1) = 0.1;
d_ugv(2, 1) = 0.12;  
d_ugv(3, 1) = 0.15;
d_ugv(4, 1) = 0.13;

% UAV自适应增益初值（降低初值）
d_uav(1, 1) = 0.08;
d_uav(2, 1) = 0.1;
d_uav(3, 1) = 0.12;
d_uav(4, 1) = 0.09;

% UGV自适应率（降低自适应率）
k_ugv = [0.01; 0.012; 0.015; 0.011];
% UAV自适应率（降低自适应率）
k_uav = [0.008; 0.01; 0.012; 0.009];

fprintf('\n=== Formation Verification ===\n');
fprintf('UGV Target Positions:\n');
for i = 1:N_ugv
    target = p_d + w(:, i);
    fprintf('UGV%d: [%.1f, %.1f]\n', i, target(1), target(2));
end

fprintf('\nUAV Target Positions:\n');
for i = 1:N_uav
    target = p_d_uav + r(:, i);
    fprintf('UAV%d: [%.1f, %.1f, %.1f]\n', i, target(1), target(2), target(3));
end

%% 主仿真循环
for k = 1:N_steps-1
    % 当前时刻状态
    x_current = x_ugv(:, :, k);  % 4xN_ugv
    y_current = y_uav(:, :, k);  % 6xN_uav
    
    % 计算目标状态
    x_d = zeros(4, N_ugv);
    y_d = zeros(6, N_uav);
    
    for i = 1:N_ugv
        x_d(:, i) = [p_d + w(:, i); 0; 0];  % 目标位置 + 队形向量
    end
    
    for i = 1:N_uav
        y_d(:, i) = [p_d_uav + r(:, i); 0; 0; 0];  % 目标位置 + 队形向量
    end
    
    % 计算控制输入和状态导数
    x_dot = zeros(4, N_ugv);
    y_dot = zeros(6, N_uav);
    
    % UGV控制器（只有基本跟踪控制，无耦合项）
    for i = 1:N_ugv
        % 计算跟踪误差
        error_x = x_current(:, i) - x_d(:, i);
        
        % 自适应控制器：u = -d * error，但只作用在加速度上
        % 将位置和速度误差都转换为加速度控制
        pos_error = error_x(1:2);  % 位置误差
        vel_error = error_x(3:4);  % 速度误差
        
        % PD型控制：u = -kp * pos_error - kd * vel_error
        % 这里用自适应增益作为总增益，然后分配给位置和速度
        kp = d_ugv(i, k) * 1.0;    % 位置增益
        kd = d_ugv(i, k) * 0.5;    % 速度增益
        
        u_total = -kp * pos_error - kd * vel_error;
        
        % 队形保持耦合项
        coupling_ugv = zeros(4, 1);
        for j = 1:N_ugv
            if A(i, j) == 1
                % 相对位置误差
                pos_error_coupling = (x_current(1:2, j) - x_current(1:2, i)) - (w(:, j) - w(:, i));
                vel_error_coupling = (x_current(3:4, j) - x_current(3:4, i));
                
                coupling_ugv(1:2) = coupling_ugv(1:2) +  pos_error_coupling;
                coupling_ugv(3:4) = coupling_ugv(3:4) +  vel_error_coupling;
            end
        end
        
        % 状态导数
        x_dot(:, i) = D_x{i} * x_current(:, i) + E_x{i} * u_total + coupling_ugv;
        
        % 自适应律（添加饱和保护）
        error_norm_sq = error_x' * error_x;
        d_ugv(i, k+1) = d_ugv(i, k) + dt * k_ugv(i) * error_norm_sq;
        % 添加合理的增益限制防止发散
        d_ugv(i, k+1) = max(0.01, min(50, d_ugv(i, k+1)));
    end
    
    % UAV控制器（只有基本跟踪控制，无耦合项）
    for i = 1:N_uav
        % 计算跟踪误差
        error_y = y_current(:, i) - y_d(:, i);
        
        % 自适应控制器：u = -d * error，但只作用在加速度上
        % 将位置和速度误差都转换为加速度控制
        pos_error = error_y(1:3);  % 位置误差 (x,y,z)
        vel_error = error_y(4:6);  % 速度误差 (vx,vy,vz)
        
        % PD型控制：u = -kp * pos_error - kd * vel_error
        kp = d_uav(i, k) * 1.0;    % 位置增益
        kd = d_uav(i, k) * 0.5;    % 速度增益
        
        u_total = -kp * pos_error - kd * vel_error;
        
        % UAV之间的队形保持耦合
        coupling_uav = zeros(6, 1);
        for j = 1:N_uav
            if B(i, j) == 1  % 只要有通信关系，耦合系数为1
                pos_error_coupling = (y_current(1:3, j) - y_current(1:3, i)) - (r(:, j) - r(:, i));
                vel_error_coupling = (y_current(4:6, j) - y_current(4:6, i));
                
                coupling_uav(1:3) = coupling_uav(1:3) + B(i, j) * pos_error_coupling;
                coupling_uav(4:6) = coupling_uav(4:6) + B(i, j) * vel_error_coupling;
            end
        end
        
        % UAV-UGV垂直耦合（高度和水平位置协调）
        for j = 1:N_ugv
            if C(i, j) == 1  % 只要有通信关系，耦合系数为1
                % 水平位置应该跟随对应的UGV
                xy_error_coupling = y_current(1:2, i) - x_current(1:2, j);
                % 高度应该保持在UGV上方2m
                z_error_coupling = y_current(3, i) - 2;
                
                coupling_uav(1:2) = coupling_uav(1:2) - C(i, j) * xy_error_coupling;
                coupling_uav(3) = coupling_uav(3) - C(i, j) * z_error_coupling;
                
                % 速度耦合
                coupling_uav(4:5) = coupling_uav(4:5) - C(i, j) * (y_current(4:5, i) - x_current(3:4, j));
                coupling_uav(6) = coupling_uav(6) - C(i, j) * y_current(6, i);
            end
        end
        
        % 状态导数（无耦合项）
        y_dot(:, i) = D_y{i} * y_current(:, i) + E_y{i} * u_total + coupling_uav;
        
        % 自适应律（添加饱和保护）
        error_norm_sq = error_y' * error_y;
        d_uav(i, k+1) = d_uav(i, k) + dt * k_uav(i) * error_norm_sq;
        % 添加合理的增益限制防止发散
        d_uav(i, k+1) = max(0.01, min(40, d_uav(i, k+1)));
    end
    
    % 数值积分更新状态
    x_ugv(:, :, k+1) = x_ugv(:, :, k) + dt * x_dot;
    y_uav(:, :, k+1) = y_uav(:, :, k) + dt * y_dot;
    
    % 添加数值稳定性检查
    if any(abs(x_ugv(:, :, k+1)) > 1000, 'all') || any(abs(y_uav(:, :, k+1)) > 1000, 'all')
        fprintf('Numerical divergence at step %d, stopping simulation\n', k);
        fprintf('Max UGV value: %.2f, Max UAV value: %.2f\n', max(abs(x_ugv(:)), [], 'all'), max(abs(y_uav(:)), [], 'all'));
        % 截断数据
        x_ugv = x_ugv(:, :, 1:k+1);
        y_uav = y_uav(:, :, 1:k+1);
        d_ugv = d_ugv(:, 1:k+1);
        d_uav = d_uav(:, 1:k+1);
        t_span = t_span(1:k+1);
        break;
    end
end

%% 计算性能指标
final_time_idx = length(t_span);

% UGV队形误差
ugv_formation_error = zeros(N_ugv, 1);
for i = 1:N_ugv
    final_pos = x_ugv(1:2, i, end);
    target_pos = p_d + w(:, i);
    ugv_formation_error(i) = norm(final_pos - target_pos);
    fprintf('UGV%d final position error: %.4f m\n', i, ugv_formation_error(i));
end

% UAV队形误差
uav_formation_error = zeros(N_uav, 1);
for i = 1:N_uav
    final_pos = y_uav(1:3, i, end);
    target_pos = p_d_uav + r(:, i);
    uav_formation_error(i) = norm(final_pos - target_pos);
    fprintf('UAV%d final position error: %.4f m\n', i, uav_formation_error(i));
end

% 最终速度
fprintf('\nUGV final velocities:\n');
for i = 1:N_ugv
    final_vel = norm(x_ugv(3:4, i, end));
    fprintf('UGV%d: %.4f m/s\n', i, final_vel);
end

fprintf('\nUAV final velocities:\n');
for i = 1:N_uav
    final_vel = norm(y_uav(4:6, i, end));
    fprintf('UAV%d: %.4f m/s\n', i, final_vel);
end

%% 绘制误差分析图
% 定义颜色
colors = {'r', 'g', 'b', 'm'};

% 计算各维度误差
ugv_pos_errors_x = zeros(N_ugv, length(t_span));
ugv_pos_errors_y = zeros(N_ugv, length(t_span));
ugv_vel_errors_x = zeros(N_ugv, length(t_span));
ugv_vel_errors_y = zeros(N_ugv, length(t_span));

uav_pos_errors_x = zeros(N_uav, length(t_span));
uav_pos_errors_y = zeros(N_uav, length(t_span));
uav_pos_errors_z = zeros(N_uav, length(t_span));
uav_vel_errors_x = zeros(N_uav, length(t_span));
uav_vel_errors_y = zeros(N_uav, length(t_span));
uav_vel_errors_z = zeros(N_uav, length(t_span));

height_errors = zeros(N_uav, length(t_span));

for k = 1:length(t_span)
    % UGV误差计算
    for i = 1:N_ugv
        % 位置误差：pi - pd - 队形向量 (每个维度)
        target_pos = p_d + w(:, i);
        ugv_pos_errors_x(i, k) = x_ugv(1, i, k) - target_pos(1);  % x维度误差
        ugv_pos_errors_y(i, k) = x_ugv(2, i, k) - target_pos(2);  % y维度误差
        
        % 速度误差：vi - vd (vd = 0, 每个维度)
        ugv_vel_errors_x(i, k) = x_ugv(3, i, k) - 0;  % vx误差
        ugv_vel_errors_y(i, k) = x_ugv(4, i, k) - 0;  % vy误差
    end
    
    % UAV误差计算
    for i = 1:N_uav
        % 位置误差：pi - pd - 队形向量 (每个维度)
        target_pos = p_d_uav + r(:, i);
        uav_pos_errors_x(i, k) = y_uav(1, i, k) - target_pos(1);  % x维度误差
        uav_pos_errors_y(i, k) = y_uav(2, i, k) - target_pos(2);  % y维度误差
        uav_pos_errors_z(i, k) = y_uav(3, i, k) - target_pos(3);  % z维度误差
        
        % 速度误差：vi - vd (vd = 0, 每个维度)
        uav_vel_errors_x(i, k) = y_uav(4, i, k) - 0;  % vx误差
        uav_vel_errors_y(i, k) = y_uav(5, i, k) - 0;  % vy误差
        uav_vel_errors_z(i, k) = y_uav(6, i, k) - 0;  % vz误差
        
        % UAV-UGV高度差误差 (期望高度差为2m)
        corresponding_ugv_z = 0; % UGV在地面
        actual_height_diff = y_uav(3, i, k) - corresponding_ugv_z;
        height_errors(i, k) = actual_height_diff - 2; % 期望高度差为2m，误差为实际值-期望值
    end
end

% 1. UGV位置误差图 (X和Y维度)
figure(1);
subplot(2,2,1);
for i = 1:N_ugv
    plot(t_span, ugv_pos_errors_x(i, :), colors{i}, 'LineWidth', 2, 'DisplayName', ['UGV', num2str(i)]);
    hold on;
end
yline(0, 'k--', 'LineWidth', 1);
xlabel('t(s)'); ylabel('e^x_{p1}(m)'); grid on;
legend('UGV1', 'UGV2', 'UGV3', 'UGV4', 'Location', 'best');
set(gca, 'XGrid', 'off', 'YGrid', 'off');

subplot(2,2,2);
for i = 1:N_ugv
    plot(t_span, ugv_pos_errors_y(i, :), colors{i}, 'LineWidth', 2, 'DisplayName', ['UGV', num2str(i)]);
    hold on;
end
yline(0, 'k--', 'LineWidth', 1);
xlabel('t(s)'); ylabel('e^x_{p2}(m)'); grid on;
legend('UGV1', 'UGV2', 'UGV3', 'UGV4', 'Location', 'best');
set(gca, 'XGrid', 'off', 'YGrid', 'off');

% UGV速度误差图 (X和Y维度)
subplot(2,2,3);
for i = 1:N_ugv
    plot(t_span, ugv_vel_errors_x(i, :), colors{i}, 'LineWidth', 2, 'DisplayName', ['UGV', num2str(i)]);
    hold on;
end
yline(0, 'k--', 'LineWidth', 1);
xlabel('t(s)'); ylabel('e^x_{v1}(m/s)'); grid on;
legend('UGV1', 'UGV2', 'UGV3', 'UGV4', 'Location', 'best');
set(gca, 'XGrid', 'off', 'YGrid', 'off');

subplot(2,2,4);
for i = 1:N_ugv
    plot(t_span, ugv_vel_errors_y(i, :), colors{i}, 'LineWidth', 2, 'DisplayName', ['UGV', num2str(i)]);
    hold on;
end
yline(0, 'k--', 'LineWidth', 1);
xlabel('t(s)'); ylabel('ev_{v2}(m/s)'); grid on;
legend('UGV1', 'UGV2', 'UGV3', 'UGV4', 'Location', 'best');
set(gca, 'XGrid', 'off', 'YGrid', 'off');

% 2. UAV位置误差图 (X, Y, Z维度)
figure(2);
subplot(2,3,1);
for i = 1:N_uav
    plot(t_span, uav_pos_errors_x(i, :), colors{i}, 'LineWidth', 2, 'DisplayName', ['UAV', num2str(i)]);
    hold on;
end
yline(0, 'k--', 'LineWidth', 1);
xlabel('t(s)'); ylabel('e^y_{p1}(m)'); grid on;
legend('UAV1', 'UAV2', 'UAV3', 'UAV4', 'Location', 'best');
set(gca, 'XGrid', 'off', 'YGrid', 'off');

subplot(2,3,2);
for i = 1:N_uav
    plot(t_span, uav_pos_errors_y(i, :), colors{i}, 'LineWidth', 2, 'DisplayName', ['UAV', num2str(i)]);
    hold on;
end
yline(0, 'k--', 'LineWidth', 1);
xlabel('t(s)'); ylabel('e^y_{p2}(m)'); grid on;
legend('UAV1', 'UAV2', 'UAV3', 'UAV4', 'Location', 'best');
set(gca, 'XGrid', 'off', 'YGrid', 'off');

subplot(2,3,3);
for i = 1:N_uav
    plot(t_span, uav_pos_errors_z(i, :), colors{i}, 'LineWidth', 2, 'DisplayName', ['UAV', num2str(i)]);
    hold on;
end
yline(0, 'k--', 'LineWidth', 1);
xlabel('t(s)'); ylabel('e^y_{p3}(m)'); grid on;
legend('UAV1', 'UAV2', 'UAV3', 'UAV4', 'Location', 'best');
set(gca, 'XGrid', 'off', 'YGrid', 'off');

% UAV速度误差图 (X, Y, Z维度)
subplot(2,3,4);
for i = 1:N_uav
    plot(t_span, uav_vel_errors_x(i, :), colors{i}, 'LineWidth', 2, 'DisplayName', ['UAV', num2str(i)]);
    hold on;
end
yline(0, 'k--', 'LineWidth', 1);
xlabel('t(s)'); ylabel('e^y_{v1}(m/s)'); grid on;
legend('UAV1', 'UAV2', 'UAV3', 'UAV4', 'Location', 'best');
set(gca, 'XGrid', 'off', 'YGrid', 'off');

subplot(2,3,5);
for i = 1:N_uav
    plot(t_span, uav_vel_errors_y(i, :), colors{i}, 'LineWidth', 2, 'DisplayName', ['UAV', num2str(i)]);
    hold on;
end
yline(0, 'k--', 'LineWidth', 1);
xlabel('t(s)'); ylabel('e^y_{v}(m/s)'); grid on;
legend('UAV1', 'UAV2', 'UAV3', 'UAV4', 'Location', 'best');
set(gca, 'XGrid', 'off', 'YGrid', 'off');

subplot(2,3,6);
for i = 1:N_uav
    plot(t_span, uav_vel_errors_z(i, :), colors{i}, 'LineWidth', 2, 'DisplayName', ['UAV', num2str(i)]);
    hold on;
end
yline(0, 'k--', 'LineWidth', 1);
xlabel('t(s)'); ylabel('e^y_{v3}(m/s)'); grid on;
legend('UAV1', 'UAV2', 'UAV3', 'UAV4', 'Location', 'best');
set(gca, 'XGrid', 'off', 'YGrid', 'off');

% 3. 自适应增益图
figure(3);
for i = 1:N_ugv
    plot(t_span, d_ugv(i, :), colors{i}, 'LineWidth', 2);
    hold on;
end
xlabel('t(s)'); ylabel('d_i^x'); grid on;
legend('UGV1', 'UGV2', 'UGV3', 'UGV4');
set(gca, 'XGrid', 'off', 'YGrid', 'off');

figure(4);
for i = 1:N_uav
    plot(t_span, d_uav(i, :), colors{i}, 'LineWidth', 2);
    hold on;
end
xlabel('t(s)'); ylabel('d_i^y'); grid on;
legend('UAV1', 'UAV2', 'UAV3', 'UAV4');
set(gca, 'XGrid', 'off', 'YGrid', 'off');
%% 显示系统矩阵
fprintf('\n=== System Matrices ===\n');
fprintf('UGV state matrix example D_x{1}:\n');
disp(D_x{1});
fprintf('UGV control matrix E_x{1}:\n');
disp(E_x{1});
fprintf('UAV state matrix example D_y{1}:\n');
disp(D_y{1});
fprintf('UAV control matrix E_y{1}:\n');
disp(E_y{1});

fprintf('\nCommunication topology matrices:\n');
fprintf('A (UGV): '); disp(A);
fprintf('B (UAV): '); disp(B);  
fprintf('C (UGV-UAV): '); disp(C);