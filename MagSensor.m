% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Purpose: Class used to simulate Magnetometer sensor
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

classdef MagSensor < ADS
    properties (Constant)
        mag_error = 1.25e-7;      % Magnetometer error [T]
    end

    properties
        Mag_B;      % B in Body frame [T]
        Mag_I;      % B in Orbit (Intertial) frame [T]
    end

    % Create instance of class
    methods
        function obj = MagSensor(Data, Mag, Earth, Sun)
            obj@ADS(Data, Mag, Earth, Sun);
        end
    end

    % Generate readings if sensor is activated
    methods
        function [B_flag] = simMag(obj, kc_kal)
            B_flag = obj.Mag;
            
            if obj.Mag == 1
                obj.Mag_B = (obj.Data.B_B_STK(kc_kal,:)*1e-9)' + obj.mag_error * randn * ones(3,1);
                obj.Mag_I = (obj.Data.B_I_STK(kc_kal,:)*1e-9)';
            end
        end
    end
end