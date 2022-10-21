% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Purpose: Class used to simulate Coarse Sun sensor
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
classdef SunSensor < ADS
    properties (Constant)
        ss_error = 0.3;     % Sun sensor error [deg]
    end

    properties
        Sun_I;  % Inertial frame vector
        Sun_B;  % Body frame vector
    end

    % Create instance of class
    methods
        function obj = SunSensor(Data, Mag, Earth, Sun)
            obj@ADS(Data, Mag, Earth, Sun);
        end
    end

    % Generate readings from sensor
    methods
        function [S_flag] = simSun(obj, kc_kal)
            S_flag = 0;
            vec_B = obj.Data.S_B_STK(kc_kal,:);
            vec_I = obj.Data.S_I_STK(kc_kal,:);
            z = vec_B(3);

            if z > 0
                S_flag = 1;
                obj.Sun_B = vec_B' + (obj.ss_error) * pi/180 * randn(3,1);
            else
                S_flag = 0;
                obj.Sun_B = zeros(3,1);
            end

            obj.Sun_I = vec_I';
            S_flag = S_flag & obj.Sun;
        end
    end
end