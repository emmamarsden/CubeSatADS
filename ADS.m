% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Class used to simulate ACS and environment
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

classdef ADS < handle
    properties (Constant)
        T = 0.1;                      % IMU integration time
        T_D = 1;                      % Kalman filter discrete time
        G = 6.67428e-11;              % Earth's gravitational constant
        M = 5.972e24;                 % Earth's mass [kg]
        mu = 398.6004418e12;          % [m^3/s^2]
        Re = 6.378137e6;              % Earth's radius [m]
        w_IE = 7.292115147e-5;        % Earth's rotation velocity [rad/s]
        Rs = 350e3;                   % Satellite altitude [m]
    end

    properties (Dependent)
        Rc          % Distance from Earth center to satellite [m]
        w_o         % Satellite angular velocity relative to Earth [rad/s]
        Period      % Satellite angular velocity relative to Earth [s]
    end

    properties
        Data;                   % Database files
        Mag;                    % Magnetometer [1 = activated]
        Earth;                  % Earth sensor [1 = activated]
        Sun;                    % Sun sensor [1 = activated]
        atti = [45;0;0];        % Initial attitude [R-P-Y][deg]
        atti_rate = [1;2;3];    % [deg/s]
    end

    % Create instance of object
    methods
        function obj = ADS(Data, Mag, Earth, Sun)
            if nargin < 4
                obj.Mag = 1;
                obj.Earth = 1;
                obj.Sun = 1;
            else
                obj.Mag = Mag;
                obj.Earth = Earth;
                obj.Sun = Sun;
            end
            obj.Data = load(Data);
        end
    end

    % Get and Set methods
    methods
        function Rc = get.Rc(obj)
            Rc = obj.Re + obj.Rs;
        end

        function w_o = get.w_o(obj)
            w_o = sqrt(obj.G * obj.M/obj.Rc^3);
        end

        function Period = get.Period(obj)
            Period = (2*pi)/(obj.w_o);
        end
    end

end