% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Purpose: Class used to simulate Earth sensor
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

classdef EarthSensor < ADS
    properties (Constant)
        es_error = 0.5;   % Earth sensor error [rad]
    end

    properties
        Earth_B;    % Body frame vector
        Earth_I;    % Inertial frame vector
    end

    % Create instance of class
    methods
        function obj = EarthSensor(Data, Mag, Earth, Sun)
            obj@ADS(Data, Mag, Earth, Sun);
        end
    end

    % Generate readings if sensor is activated
    methods
        function [E_flag] = simEarth(obj, kc_kal)
            E_flag = obj.Earth;
            EarthB = [];
            EarthI = [];

            if obj.Earth == 1
                obj.Earth_B = obj.Data.E_B_STK(kc_kal,:)' + (obj.es_error) * pi/180 * randn * ones(3,1);
                obj.Earth_I = obj.Data.E_I_STK(kc_kal,:)';
            end
        end
    end
end

