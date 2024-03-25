frc_data = dlmread('record_data.txt');
frc.time = frc_data(:, 1);
frc.pos = frc_data(:, 2:3);
frc.posd = frc_data(:, 4:5);
frc.delta = frc_data(:, 6:7);
frc.absyz = frc_data(:, 8:9);
frc.fyz = frc_data(:, 10:11);

figure(1); % �½�һ��ͼ�δ���
hold on; % �������֣�������ͬһͼ�ϻ��ƶ�������

% ���� pos ����
plot(frc.time, frc.pos(:, 1), 'b-', 'LineWidth', 1); % posy ʹ����ɫʵ��
plot(frc.time, frc.pos(:, 2), 'b--', 'LineWidth', 1); % posz ʹ����ɫ����

% ���� posd ����
plot(frc.time, frc.posd(:, 1), 'r-', 'LineWidth', 1); % posdy ʹ�ú�ɫʵ��
plot(frc.time, frc.posd(:, 2), 'r--', 'LineWidth', 1); % posdz ʹ�ú�ɫ����

legend('posy', 'posz', 'posdy', 'posdz'); % ���ͼ��
xlabel('Time'); % x���ǩ
ylabel('Position and Position Derivative'); % y���ǩ
title('Position and Position Derivative Over Time'); % ͼ�ı���
hold off; % �رձ���

figure(2); % �½���һ��ͼ�δ���
plot(frc.time, frc.delta, 'LineWidth', 1); % ���� delta ����
legend('deltay', 'deltaz'); % ���ͼ����������1��deltay����2��deltaz
xlabel('Time'); % x���ǩ
ylabel('Delta'); % y���ǩ
title('Delta Over Time'); % ͼ�ı���

frc.delta_scaled = frc.delta * 100; % ��delta���ݳ���100

figure(3); % �����ڶ���ͼ�δ���
hold on; % �������ֵ�ǰͼ��
plot(frc.time, frc.delta_scaled, 'linewidth', 1); % ���Ƴ���100���delta����
plot(frc.time, frc.fyz, '--', 'linewidth', 1); % ����fyz���ݣ�ʹ�������Ա�����
hold off; % �رձ���״̬
legend('deltay * 100', 'deltaz * 100', 'fy', 'fz'); % ���ͼ��
title('Delta and Forces'); % ��ӱ���
xlabel('Time'); % x���ǩ
ylabel('Scaled Value / Force'); % y���ǩ


