frc_data = dlmread('record_data1.txt');
frc.time = frc_data(:, 1);
frc.pos = frc_data(:, 2:3);
frc.posd = frc_data(:, 4:5);
frc.delta = frc_data(:, 6:7);
frc.absyz = frc_data(:, 8:9);
frc.fyz = frc_data(:, 10:11);
frc.theta = frc_data(:, 12);

figure(1); % 新建一个图形窗口
hold on; % 开启保持，允许在同一图上绘制多条曲线

% 绘制 pos 数据
plot((frc.time - 1711386607)*100, frc.pos(:, 1)*100, 'b-', 'LineWidth', 1); % posy 使用蓝色实线
plot((frc.time - 1711386607)*100, frc.pos(:, 2)*100, 'b--', 'LineWidth', 1); % posz 使用蓝色虚线

% 绘制 posd 数据
plot((frc.time - 1711386607)*100, frc.posd(:, 1)*100, 'r-', 'LineWidth', 1); % posdy 使用红色实线
plot((frc.time - 1711386607)*100, frc.posd(:, 2)*100, 'r--', 'LineWidth', 1); % posdz 使用红色虚线

plot((frc.time - 1711386607)*100, frc.theta, 'g', 'LineWidth', 1); % posdz 使用红色虚线

% plot(frc.time - 1711386607)*100, frc.delta(:, 1)*1000, '-', 'LineWidth', 1,'Color',[0.6 0.6 0.6]); % posdy 使用红色实线
% plot(frc.time - 1711386607)*100, frc.delta(:, 2)*1000, '-', 'LineWidth', 1,'Color',[0.6 0.6 0.9]); % posdz 使用红色虚线
% 绘制 posd 数据
plot((frc.time - 1711386607)*100, frc.fyz(:, 1), '-', 'LineWidth', 1,'Color',[0.6 0.6 0.6]);% posdy 使用红色实线
plot((frc.time - 1711386607)*100, frc.fyz(:, 2), '-', 'LineWidth', 1,'Color',[0.6 0.9 0.9]); % posdz 使用红色虚线

legend('posy', 'posz', 'posdy', 'posdz', 'theta', 'fy', 'fz'); % 添加图例
xlabel('Time'); % x轴标签
ylabel('Position and Position Derivative'); % y轴标签
title('Position and Position Derivative Over Time'); % 图的标题
hold off; % 关闭保持

figure(2); % 新建另一个图形窗口
plot((frc.time - 1711386607)*100, frc.delta, 'LineWidth', 1); % 绘制 delta 数据
legend('deltay', 'deltaz'); % 添加图例，假设列1是deltay，列2是deltaz
xlabel('Time'); % x轴标签
ylabel('Delta'); % y轴标签
title('Delta Over Time'); % 图的标题

frc.delta_scaled = frc.delta * 100; % 将delta数据乘以100

figure(3); % 创建第二个图形窗口
hold on; % 开启保持当前图形
plot((frc.time - 1711386607)*100, frc.delta_scaled, 'linewidth', 1); % 绘制乘以100后的delta数据
plot((frc.time - 1711386607)*100, frc.fyz, '--', 'linewidth', 1); % 绘制fyz数据，使用虚线以便区分
hold off; % 关闭保持状态
legend('deltay * 100', 'deltaz * 100', 'fy', 'fz'); % 添加图例
title('Delta and Forces'); % 添加标题
xlabel('Time'); % x轴标签
ylabel('Scaled Value / Force'); % y轴标签


