% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Purpose:
% Plots results of Attitude Determination
% 
% Inputs:
% N/A
%
% Generates:
% N/A
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
close all;
Data = open('STKData.mat')
len =  length(SinsData);
RPY_ture = zeros(len,3);
RPY_sins =  zeros(len,3);

% % % Figure 1
figure
subplot(3,1,1)
plot(Data.Q_B_IB_STK(1:len,1:4),'LineWidth',2)
ylabel('Actu.Quat.')
title('Actual Q^B_I_B')

subplot(3,1,2)
plot(SinsData(:,1)/K.Period,SinsData(:,2:5),'LineWidth',2)
ylabel('Esti.Quat.')
title('Gyro')

subplot(3,1,3)
plot(KalStates(:,1)/K.Period,KalStates(:,2:5),'LineWidth',2)
ylabel('Esti.Quat.')
title('Estimated')
xlabel('Orbits')

% % % Figure 2
for i = 1:len
    RPY_ture(i,:) = Q2RPY312(Data.Q_B_IB_STK(i,:))';
    RPY_sins(i,:) = Q2RPY312(SinsData(i,2:5))';
end

err_euler = RPY_ture-RPY_sins;

figure
subplot(3,3,1)
plot(SinsData(:,1)/K.Period,RPY_ture(:,1),'LineWidth',2)
ylabel('R')
title('Actual Euler^B_I (deg)')

subplot(3,3,2)
plot(SinsData(:,1)/K.Period,RPY_sins(:,1),'LineWidth',2)
ylabel('Gyro R')
title('Gyro Euler^B_I (deg)')

subplot(3,3,3)
plot(SinsData(:,1)/K.Period,err_euler(:,1),'LineWidth',2)
ylabel('Error R')
title('Error Euler^B_I (Sins vs. True)(deg)')

subplot(3,3,4)
plot(SinsData(:,1)/K.Period,RPY_ture(:,2),'LineWidth',2)
ylabel('P')

subplot(3,3,5)
plot(SinsData(:,1)/K.Period,RPY_sins(:,2),'LineWidth',2)
ylabel('Gyro P')

subplot(3,3,6)
plot(SinsData(:,1)/K.Period,err_euler(:,2),'LineWidth',2)
ylabel('Error P')

subplot(3,3,7)
plot(SinsData(:,1)/K.Period,RPY_ture(:,3),'LineWidth',2)
ylabel('Y')
xlabel('Orbits');

subplot(3,3,8)
plot(SinsData(:,1)/K.Period,RPY_sins(:,3),'LineWidth',2)
ylabel('Gyro Y')
xlabel('Orbits')

subplot(3,3,9)
plot(SinsData(:,1)/K.Period,err_euler(:,3),'LineWidth',2)
ylabel('Error Y')
xlabel('Orbits')

% % % Figure 3 
for i = 1:len
    Euler_ture_B_O(i,:) = Q2RPY312(Q2A(Data.Q_B_IB_STK(i,:)')*Q2A(Data.Q_O_I_STK(i,:)')');
    Euler_sins_B_O(i,:) = Q2RPY312(Q2A(SinsData(i,2:5)')*Q2A(Data.Q_O_I_STK(i,:)')');
end

save Euler_ture_B_O.mat Euler_ture_B_O;
save Euler_sins_B_O.mat Euler_sins_B_O;

figure
subplot(3,2,1)
plot(SinsData(:,1)/K.Period, Euler_ture_B_O(:,1),'LineWidth',2)
ylabel('R')
title('Ture RPY^B_O (deg)')

subplot(3,2,2)
plot(SinsData(:,1)/K.Period, Euler_sins_B_O(:,1),'LineWidth',2)
ylabel('R')
title('Gyro RPY^B_O (deg)')

subplot(3,2,3)
plot(SinsData(:,1)/K.Period,Euler_ture_B_O(:,2),'LineWidth',2)
ylabel('P')

subplot(3,2,4)
plot(SinsData(:,1)/K.Period,Euler_sins_B_O(:,2),'LineWidth',2)
ylabel('P')

subplot(3,2,5)
plot(SinsData(:,1)/K.Period,Euler_ture_B_O(:,3),'LineWidth',2)
ylabel('Y')
xlabel('Orbits')

subplot(3,2,6)
plot(SinsData(:,1)/K.Period,Euler_sins_B_O(:,3),'LineWidth',2)
ylabel('Y')
xlabel('Orbits')

% % % Figure 4
figure
subplot(3,1,1)
plot(SinsData(:,1)/K.Period,Euler_sins_B_O(:,1)-Euler_ture_B_O(:,1),'LineWidth',2)
grid
ylabel('Err R')
ylim([-2 2])
title('Error RPY^B_O(deg)')

subplot(3,1,2)
plot(SinsData(:,1)/K.Period,Euler_sins_B_O(:,2)-Euler_ture_B_O(:,2),'LineWidth',2)
grid
ylabel('Err P')
ylim([-2 2])

subplot(3,1,3)
plot(SinsData(:,1)/K.Period,Euler_ture_B_O(:,3)-Euler_sins_B_O(:,3),'LineWidth',2)
grid
ylabel('Err Y')
ylim([-2 2])
xlabel('Orbits')

Err_Euler_B_O = Euler_sins_B_O-Euler_ture_B_O;
for i = 1:length(Err_Euler_B_O)
    Err_norm(i,1) = norm(Err_Euler_B_O(i,:));   
end

% % % Figure 5
figure
plot(SinsData(:,1)/K.Period,Err_norm,'LineWidth',2)
ylabel('Error Euler(deg)')
ylim([0 10])
hold on

plot([SinsData(1,1)/K.Period,SinsData(end,1)/K.Period],[2 2],'r--','LineWidth',2)
hold off
xlabel('Orbits')
grid on

% % % Figure 6
figure
subplot(3,2,1)
plot(GyroData(:,1)/K.Period, rad2deg(GyroData(:,2)),'LineWidth',2)
ylabel('x')
grid
title('Gyro output vs. Actual (deg/s)')
hold on
plot(GyroData(:,1)/K.Period,Data.w_B_IB_STK(1:len,1),'r','LineWidth',2)
hold off
grid
legend('Gyro','Actual')

subplot(3,2,2)
plot(GyroData(:,1)/K.Period,rad2deg(GyroData(:,5))*3600,'LineWidth',2)
ylabel('x')
grid
title('Gyro bias (deg)')

subplot(3,2,3)
plot(GyroData(:,1)/K.Period, rad2deg(GyroData(:,3)),'LineWidth',2)
ylabel('y')
grid
hold on
plot(GyroData(:,1)/K.Period,Data.w_B_IB_STK(1:len,2),'r','LineWidth',2)
grid
hold off

subplot(3,2,4)
plot(GyroData(:,1)/K.Period, rad2deg(GyroData(:,6))*3600,'LineWidth',2)
ylabel('y')
grid

subplot(3,2,5)
plot(GyroData(:,1)/K.Period, rad2deg(GyroData(:,4)),'LineWidth',2)
ylabel('z')
grid
hold on
plot(GyroData(:,1)/K.Period,Data.w_B_IB_STK(1:len,3),'r','LineWidth',2)
hold off
xlabel('Orbits')
grid

subplot(3,2,6)
plot(GyroData(:,1)/K.Period, rad2deg(GyroData(:,7))*3600,'LineWidth',2)
ylabel('z')
grid
xlabel('Orbits')

% % % Figure 7
for i=1:length(KalData)
    norm_P_1(i,1) = norm(KalData(i,2:4));
    norm_P_2(i,2) = norm(KalData(i,5:7));    
end

figure
subplot(2,4,1)
plot(rad2deg(ErrData(:,2:4)),'LineWidth',2)
ylim([-5 5])
ylabel('Res. Angle (deg)')
grid
title('EKF states')

subplot(2,4,2)
plot(KalData(:,1), norm_P_1,'LineWidth',2)
ylabel('||P1||')
grid
title('Error Covariance Matrix Normal')

subplot(2,4,3)
plot(ErrData(:,1),ErrData(:,8),'LineWidth',2)
ylabel('EKF')
ylim([0 2])
grid
title('EKF flag')

subplot(2,4,4)
plot(FlagData(:,3),'LineWidth',2)
ylabel('Sun')
title('CubeSense flag')
grid
ylim([0 2])

subplot(2,4,5)
plot(rad2deg(ErrData(:,5:7)),'LineWidth',2)
ylim([-0.03 +0.03])
ylabel('Res. Bias (deg/s)')
grid
xlabel('Time (s)')

subplot(2,4,6)
plot(KalData(:,1), norm_P_2,'LineWidth',2)
ylabel('||P2||')
grid
xlabel('Time (s)')

subplot(2,4,7)
plot(FlagData(:,2),'LineWidth',2)
ylabel('Magnetic')
grid
ylim([0 2])

subplot(2,4,8)
plot(FlagData(:,4),'LineWidth',2)
ylabel('Earth')
xlabel('Time (s)')
grid
ylim([0 2])

% % % Figure 8
figure
subplot(2,1,1)
plot(KalData(:,1)/K.Period, norm_P_1,'LineWidth',2)
ylabel('||P1||')
grid
title('Error Covariance Matrix Normal')
hold on

subplot(2,1,2)
plot(KalData(:,1)/K.Period,FlagData(:,3),'--r','LineWidth',2)
ylabel('Sun')
title('Sun flag')
grid
ylim([0 2])
hold off
xlabel('Orbits)')

% % % Figure 9
figure
subplot(3,1,1)
plot(rad2deg(ErrData(:,2:4)),'LineWidth',2)
ylim([-5 5])
ylabel('Res. Angle (deg)')
grid
title('EKF states')
ylim([-3 3])

subplot(3,1,2)
plot(rad2deg(ErrData(:,5:7)),'LineWidth',2)
ylim([-0.03 +0.03])
ylabel('Res. Bias (deg/s)')
grid
ylim([-0.02 0.02])

subplot(3,1,3)
plot(FlagData(:,3),'LineWidth',2)
ylabel('Sun')
title('CubeSense flag')
grid;ylim([0 2])
xlabel('Time (s)')
disp('---> End plot results')