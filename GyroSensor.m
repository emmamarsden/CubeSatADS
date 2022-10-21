% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Purpose: Class used to simulate Gyrometer
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

classdef GyroSensor < ADS
    properties
        Gyro_b = zeros(3,1);    % Body frame - gyro random constant bias [rad/s]
        Gyro_m = zeros(3,1);    % Gyro output [rad/s]
        ARW                     % Angular Rate Walk [rad/s]
        RRW                     % Rate Random Walk [rad/sec^(3/2)]
    end
    
    properties (Access = private)
        Kg_bias = diag([0.0 0.0 0.0]);       % Gyro scale factor error matrix
        Ag_bias = zeros(3,3);                % Misalignment error matrix
        Gyro_fix = zeros(3,1);               % Body frame gyro fixed error output [rad/s]
        Gyro_wg = zeros(3,1);                % Gyro white noise [rad/s]
    end

    % Initialise object of class
    methods
        function obj = GyroSensor(Data, Mag, Earth, Sun)
            obj@ADS(Data, Mag, Earth, Sun);

            N_ARW = deg2rad(0.029);         % [rad/s]
            K_RRW = deg2rad(0.0002);        % [rad/s]
            
            obj.ARW = sqrt(N_ARW^2);        % [rad/s]
            obj.RRW = sqrt(K_RRW^2/3);      % [rad/sec^(3/2)]
        end
    end

    % Generate sensor readings
    methods
        function obj = simGyro(obj, t, w_B_IB)
            obj.Gyro_fix = (obj.Kg_bias + obj.Ag_bias) * w_B_IB;

            if (t == 0)
                obj.Gyro_b = deg2rad(3/3600) * randn * ones(3,1);
                obj.Gyro_wg =  obj.ARW * randn * ones(3,1);
            else
                obj.Gyro_b  =  obj.Gyro_b + obj.T * obj.RRW * randn * ones(3,1);
                obj.Gyro_wg =  obj.ARW * randn * ones(3,1);
            end

            obj.Gyro_m  =  w_B_IB + obj.Gyro_fix + obj.Gyro_b + obj.Gyro_wg;
        end
    end

end
