% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Purpose: Back-end MATLAB file to act as Simulink workspace
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
clear
load("STKData.mat") % STK reference data

% Set orbital parameters
M = 5.972e24;               % Earth's mass [kg]
G = 6.67428e-11;            % Gravitational constant
Rs = 350e3;                 % Satellite altitude [m]
Re = 6.378137e6;            % Earth's radius [m]
Rc = Re + Rs;               % Distance from Earth center to satellite [m]
w_o = sqrt(G * M/Rc^3);     % Angular velocity relative to Earth [rad/s]
Period = (2*pi)/(w_o);      % Angular velocity relative to Earth [s]


% Initialise time-related variables
time = 0;                   % Time variable
t_0 = 0;                    % Initial time
T = 0.1;                    % IMU integration time
t_stop = fix(3*Period);     % Indicates end of simulatoin

% Set sensor errors
mag_error = 1.25e-7;
es_error = 0.5;
ss_error = 0.3; 

N_ARW = deg2rad(0.029);     % [rad/s]
K_RRW = deg2rad(0.0002);    % [rad/s]
ARW = sqrt(N_ARW^2);        % Angular Rate Walk [rad/s]
RRW = sqrt(K_RRW^2/3);      % Rate Random Walk [rad/sec^(3/2)]

% Set iteration counters
kc_kal = 1;
kc_gyro = 1;

% Initialise values for each sensor
MagB.signals.values = B_B_STK(kc_kal,:);
MagB.time = [0];
MagI.signals.values = B_I_STK(kc_kal,:);
MagI.time = [0];

EarthB.signals.values = E_B_STK(kc_kal,:);
EarthB.time = [0];
EarthI.signals.values = E_I_STK(kc_kal,:);
EarthI.time = [0];

SunB.signals.values = S_B_STK(kc_kal,:);
SunB.time = [0];
SunI.signals.values = S_I_STK(kc_kal,:);
SunI.time = [0];
z = SunB.signals.values(3);

w_B_IB.signals.values = deg2rad(w_B_IB_STK(kc_gyro,:));
w_B_IB.time = [0];
GyroB = deg2rad(3/3600) * randn * ones(3,1);

% Run Simulink simulations, run in loop if required
sim("ADS.slx");
