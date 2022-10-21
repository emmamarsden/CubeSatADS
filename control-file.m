% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Purpose:
% Attitude Determination System based on EKF
% 1) IMU Gyro
% 2) Magnetometer
% 3) Sun Sensor
% 4) Earth Sensor
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
clear
close all
clc
randn('state', 0)

K = KalmanFilter('STKData.mat', 1, 1, 1);

GyroData = zeros(K.t_stop/K.T,8);
SinsData = zeros(K.t_stop/K.T,5);
KalData = zeros(K.t_stop,8);
ErrData = zeros(K.t_stop,8);

t = 0;
kc_gyro = 1;
kc_kal = 1;
time = 0;
t_0 = 0;

MagI = [];
MagB = [];
SunB = [];
SunI = [];
EarthB = [];
EarthI = [];

for t = t_0:K.T:K.t_stop
    % Replaces sensor_Gyro.m
    w_B_IB = deg2rad(K.Data.w_B_IB_STK(kc_gyro,:)');
    K.simGyro(t, w_B_IB);

    if t == 0
        % Replaces kal_atti_init.m
        K.initKalman;

        K.bias = K.X_k(5:7);
        KalData(kc_kal, :) = [t, K.P_k(1,1), K.P_k(2,3), K.P_k(3,3), K.P_k(4,4), K.P_k(5,5), K.P_k(6,6), kc_kal];
        Q_B_IB_Sins = K.Data.Q_B_IB_STK(kc_gyro,:)'; % Initial Q value
    end

    % Update loop and check attitude sensors
    if (time == 10) % every 1 second
        time = 0;

        % Replaces sensor_magnet.m, sensor_sun.m and sensor_earth.m
        [B_flag] = K.simMag(kc_kal);
        [S_flag] = K.simSun(kc_kal);
        [E_flag] = K.simEarth(kc_kal);

        MagB = [MagB; K.Mag_B'];
        MagI = [MagI; K.Mag_I'];
        
        SunB = [SunB; K.Sun_B'];
        SunI = [SunI; K.Sun_I'];

        EarthB = [EarthB; K.Earth_B'];
        EarthI = [EarthI; K.Earth_I'];

        % Replaces kal_atti.m
        flags = [B_flag, S_flag, E_flag];
        [Xerr] = K.runKalman(flags);

        kc_kal = kc_kal + 1;
        Q_B_IB_Kal = K.X_k(1:4); 
        K.bias = K.X_k(5:7);
        Q_B_IB_Sins = Q_B_IB_Kal;

        KalStates(kc_kal,:) = [t, K.X_k'];
        KalData(kc_kal,:)   = [t, K.P_k(1,1), K.P_k(2,3), K.P_k(3,3), K.P_k(4,4), K.P_k(5,5), K.P_k(6,6), kc_kal];
        ErrData(kc_kal,:)   = [t, Xerr(1), Xerr(2), Xerr(3), Xerr(4), Xerr(5), Xerr(6), K.Kalman];
        FlagData(kc_kal,:)  = [t, B_flag, S_flag, E_flag];
    end

    % Replaces atti_cal_cq
    [Q_B_IB_Sins] = K.updateKalman(Q_B_IB_Sins);

    GyroData(kc_gyro,:) = [t, K.Gyro_m', K.Gyro_b', kc_gyro];
    SinsData(kc_gyro,:) = [t, Q_B_IB_Sins'];
    kc_gyro = kc_gyro + 1;
    time = time + 1 ;   % Propagates time
end

save Gyro1.dat GyroData -ASCII
save Sins1.dat SinsData -ASCII
save KalData1.dat KalData -ASCII
save Err1.dat ErrData -ASCII
save Flags1.dat FlagData -ASCII
save KalStates1.dat KalStates -ASCII

plot_res