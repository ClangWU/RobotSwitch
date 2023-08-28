clc;
clear;
close all;
addpath('imu_intergration_library');
%重力产生的加速度矢量
g=9.79;
G=[0,0,-g]';

%****************************读入数据
%**********读入陀螺仪的数据
%  gyro_x = xlsread('mti3_jingzhi10s.xlsx','L2:L1098');
%  gyro_y = xlsread('mti3_jingzhi10s.xlsx','M2:M1098');
%  gyro_z = xlsread('mti3_jingzhi10s.xlsx','N2:N1098');

%****************读入线性加速度计的数据****************
%测试数据
% FreeAcc_x = xlsread('zero_test.xlsx','F2:F100');      %  0 0 0 0 
% FreeAcc_y = xlsread('zero_test.xlsx','G2:G100');      %  1 2 3 4 5
% FreeAcc_z = xlsread('zero_test.xlsx','H2:H100');      %  0 0 0 0 0
% %********************************
%% Import and Process data
ahrs_static_data = dlmread('ahrs_static.txt');
ahrs.Time    = ahrs_static_data(:, 1);
ahrs.rawacc  = ahrs_static_data(:, 2:4);
ahrs.realacc = ahrs_static_data(:, 5:7);
ahrs.pos     = ahrs_static_data(:, 8:10);
ahrs.quaternion  = ahrs_static_data(:, 11:14);
% FreeAcc_x = xlsread('mti3_jingzhi10s.xlsx','F2:F1098');     %mti3静止10S数据
% FreeAcc_y = xlsread('mti3_jingzhi10s.xlsx','G2:G1098');
% FreeAcc_z = xlsread('mti3_jingzhi10s.xlsx','H2:H1098');
FreeAcc_x = ahrs.realacc(:,1);     
FreeAcc_y = ahrs.realacc(:,2); 
FreeAcc_z = ahrs.realacc(:,3); 

% FreeAcc_x = xlsread('mti3_jingzhi10min.xlsx','F2:F59344');  %mti3静止10min数据
% FreeAcc_y = xlsread('mti3_jingzhi10min.xlsx','G2:G59344');
% FreeAcc_z = xlsread('mti3_jingzhi10min.xlsx','H2:H59344');


% FreeAcc_x = xlsread('mti3_jiasujiansu.xlsx','F2:F3072');  %mti3加速减速数据
% FreeAcc_y = xlsread('mti3_jiasujiansu.xlsx','G2:G3072');
% FreeAcc_z = xlsread('mti3_jiasujiansu.xlsx','H2:H3072');

% FreeAcc_x = xlsread('lpms_jingzhi10min.xlsx','T2:T32185');     %lpms静止10min数据
% FreeAcc_y = xlsread('lpms_jingzhi10min.xlsx','U2:U32185');
% FreeAcc_z = xlsread('lpms_jingzhi10min.xlsx','V2:V32185');
% FreeAcc_x = FreeAcc_x*g;     % lpms加速度单位是g 需要先统一单位
% FreeAcc_y = FreeAcc_y*g;
% FreeAcc_z = FreeAcc_z*g;    

%*******************放进同一个矩阵，三行***********************************
FreeAcc=[FreeAcc_x';FreeAcc_y';FreeAcc_z'];  %    FreeAcc  A代表原始数据
%*******************去均值***********************************
% Free_aver_acc_x=mean(FreeAcc_x);
% Free_aver_acc_y=mean(FreeAcc_y);
% Free_aver_acc_z=mean(FreeAcc_z);
% Freeacc(1,:)=FreeAcc(1,:)-Free_aver_acc_x;   %   Freeacc  a代表去均值数据
% Freeacc(2,:)=FreeAcc(2,:)-Free_aver_acc_y;
% Freeacc(3,:)=FreeAcc(3,:)-Free_aver_acc_z; 

Freeacc(1,:)=FreeAcc(1,:);   %   算法中包含去均值，所以这里不需要去均值
Freeacc(2,:)=FreeAcc(2,:);
Freeacc(3,:)=FreeAcc(3,:); 

%采样时间
dtime=0.005;
tm=0:dtime:0.005* (size(FreeAcc,2)-1);

%*************************原始线性加速度采样曲线*****************************
figure
subplot(2,1,1);
plot(tm,FreeAcc_x,'r-',tm,FreeAcc_y,'g-',tm,FreeAcc_z,'b-.');
title('原始线性加速度采样曲线');
legend('FreeACC_X','FreeACC_Y','FreeACC_Z');
xlabel('Time / (1s)');
ylabel('Free_ACC/ (m/s'')');
grid on;

%*************************去均值后线性加速度计的采样曲线*********************
subplot(2,1,2);
plot(tm,Freeacc(1,:),'r-',tm,Freeacc(2,:),'g-',tm,Freeacc(3,:),'b-.');
title('去均值后线性加速度计的采样曲线');
legend('Freeacc_x','Freeacc_y','Freeacc_z');
xlabel('Time / (1s)');
ylabel('Freeacc/ (m/s'')');
grid on;

% 由加速度信号积分算速度位移
%*************************时域计算******************************************
[t_disint_x, t_velint_x] = IntFcn(Freeacc(1,:),tm, dtime, 1);  %  1 为时域，2为频域
[t_disint_y, t_velint_y] = IntFcn(Freeacc(2,:),tm, dtime, 1);  %  1 为时域，2为频域
[t_disint_z, t_velint_z] = IntFcn(Freeacc(3,:),tm, dtime, 1);  %  1 为时域，2为频域

linVel = [t_velint_x', t_velint_y', t_velint_z'];
order = 1;
filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(1/dtime), 'high');
linVelHP = filtfilt(b, a, linVel);
linPos = zeros(size(linVel));

for i = 2:length(linVelHP)
    linPos(i,:) = linPos(i-1,:) + linVelHP(i,:) *  dtime; 
end
linPosHP = filtfilt(b, a, linPos);

figure
subplot(2,2,1);
plot(tm,linVelHP(:,1)','r-',tm,linVelHP(:,2)','g-',tm,linVelHP(:,3)','b-');
%plot(tm,t_velint_x,'r-',tm,t_velint_y,'g-',tm,t_velint_z,'b-');

title('时域速度曲线');
legend('t_velint_x','t_velint_y','t_velint_z');
xlabel('Time / (1s)');
ylabel('t_velint/ (m/s)');
grid on;

% figure
subplot(2,2,2);
% plot(tm,t_disint_x,'r-',tm,t_disint_y,'g-',tm,t_disint_z,'b-');
plot(tm,linPosHP(:,1)','r-',tm,linPosHP(:,2)','g-',tm,linPosHP(:,3)','b-');

title('时域位移曲线');
legend('t_disint_x','t_disint_y','t_disint_z');
xlabel('Time / (1s)');
ylabel('t_disint/ (m)');
grid on;

%*************************频域计算******************************************
[f_disint_x, f_velint_x] = IntFcn(Freeacc(1,:),tm, dtime, 2);  %  1 为时域，2为频域
[f_disint_y, f_velint_y] = IntFcn(Freeacc(2,:),tm, dtime, 2);  %  1 为时域，2为频域
[f_disint_z, f_velint_z] = IntFcn(Freeacc(3,:),tm, dtime, 2);  %  1 为时域，2为频域

% figure
subplot(2,2,3);
plot(tm,f_velint_x,'r-',tm,f_velint_y,'g-',tm,f_velint_z,'b-');
title('频域速度曲线');
legend('f_velint_x','f_velint_y','f_velint_z');
xlabel('Time / (1s)');
ylabel('f_velint/ (m/s)');
grid on;

% figure
subplot(2,2,4);
plot(tm,f_disint_x,'r-',tm,f_disint_y,'g-',tm,f_disint_z,'b-');
title('频域位移曲线');
legend('f_disint_x','f_disint_y','f_disint_z');
xlabel('Time / (1s)');
ylabel('f_disint/ (m)');
grid on;



 figure
 subplot(2,1,1);
 plot3(t_disint_x,t_disint_y,t_disint_z);
 title('时域位移演示');
 xlabel('t_disint_x/（m）');
 ylabel('t_disint_y/（m）');
 zlabel('t_disint_z/（m）');
 grid on

% figure
subplot(2,1,2);
plot3(f_disint_x,f_disint_y,f_disint_z);
title('频域位移演示');
xlabel('f_disint_x/（m）');
ylabel('f_disint_y/（m）');
zlabel('f_disint_z/（m）');
grid on