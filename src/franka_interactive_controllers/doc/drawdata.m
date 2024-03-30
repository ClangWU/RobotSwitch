frc_data = dlmread('record_data1.txt');
frc.time = frc_data(:, 1);
frc.pos = frc_data(:, 2:3);
frc.posd = frc_data(:, 4:5);
frc.delta = frc_data(:, 6:7);
frc.absyz = frc_data(:, 8:9);
frc.fyz = frc_data(:, 10:11);
frc.theta = frc_data(:, 12);

figure(1); % �½�һ��ͼ�δ���
hold on; % �������֣�������ͬһͼ�ϻ��ƶ�������

% ���� pos ����
plot((frc.time - 1711386607)*100, frc.pos(:, 1)*100, 'b-', 'LineWidth', 1); % posy ʹ����ɫʵ��
plot((frc.time - 1711386607)*100, frc.pos(:, 2)*100, 'b--', 'LineWidth', 1); % posz ʹ����ɫ����

% ���� posd ����
plot((frc.time - 1711386607)*100, frc.posd(:, 1)*100, 'r-', 'LineWidth', 1); % posdy ʹ�ú�ɫʵ��
plot((frc.time - 1711386607)*100, frc.posd(:, 2)*100, 'r--', 'LineWidth', 1); % posdz ʹ�ú�ɫ����

plot((frc.time - 1711386607)*100, frc.theta, 'g', 'LineWidth', 1); % posdz ʹ�ú�ɫ����

% plot(frc.time - 1711386607)*100, frc.delta(:, 1)*1000, '-', 'LineWidth', 1,'Color',[0.6 0.6 0.6]); % posdy ʹ�ú�ɫʵ��
% plot(frc.time - 1711386607)*100, frc.delta(:, 2)*1000, '-', 'LineWidth', 1,'Color',[0.6 0.6 0.9]); % posdz ʹ�ú�ɫ����
% ���� posd ����
plot((frc.time - 1711386607)*100, frc.fyz(:, 1), '-', 'LineWidth', 1,'Color',[0.6 0.6 0.6]);% posdy ʹ�ú�ɫʵ��
plot((frc.time - 1711386607)*100, frc.fyz(:, 2), '-', 'LineWidth', 1,'Color',[0.6 0.9 0.9]); % posdz ʹ�ú�ɫ����

legend('posy', 'posz', 'posdy', 'posdz', 'theta', 'fy', 'fz'); % ���ͼ��
xlabel('Time'); % x���ǩ
ylabel('Position and Position Derivative'); % y���ǩ
title('Position and Position Derivative Over Time'); % ͼ�ı���
hold off; % �رձ���

figure(2); % �½���һ��ͼ�δ���
plot((frc.time - 1711386607)*100, frc.delta, 'LineWidth', 1); % ���� delta ����
legend('deltay', 'deltaz'); % ���ͼ����������1��deltay����2��deltaz
xlabel('Time'); % x���ǩ
ylabel('Delta'); % y���ǩ
title('Delta Over Time'); % ͼ�ı���

frc.delta_scaled = frc.delta * 100; % ��delta���ݳ���100

figure(3); % �����ڶ���ͼ�δ���
hold on; % �������ֵ�ǰͼ��
plot((frc.time - 1711386607)*100, frc.delta_scaled, 'linewidth', 1); % ���Ƴ���100���delta����
plot((frc.time - 1711386607)*100, frc.fyz, '--', 'linewidth', 1); % ����fyz���ݣ�ʹ�������Ա�����
hold off; % �رձ���״̬
legend('deltay * 100', 'deltaz * 100', 'fy', 'fz'); % ���ͼ��
title('Delta and Forces'); % ��ӱ���
xlabel('Time'); % x���ǩ
ylabel('Scaled Value / Force'); % y���ǩ


