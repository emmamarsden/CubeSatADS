% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Purpose: Class used to simulate Extended Kalman Filter
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

classdef KalmanFilter < EarthSensor & SunSensor & MagSensor & GyroSensor
    properties
        Kalman = 0; % Measurement value (1 = observations found)
    end

    properties
        t_stop;     % Simulation stop time [sec]
        bias;       % Estimated bias
        X_k;        % Updated states 
        P_k;        % Updated P matrix
        w_k;        % Gyroscope output - estimated bias states
    end

    methods
        function obj = KalmanFilter(Data, Mag, Earth, Sun)
            obj@EarthSensor(Data, Mag, Earth, Sun);
            obj@SunSensor(Data, Mag, Earth, Sun);
            obj@MagSensor(Data, Mag, Earth, Sun);
            obj@GyroSensor(Data, Mag, Earth, Sun);

            obj.t_stop = fix(3*obj.Period);
        end
    end

    methods
        function [sk] = SKEW(vec) % SKEW matrix
            sk = [0 -vec(3) vec(2);...
                vec(3) 0 -vec(1);...
                -vec(2) vec(1) 0];
        end

        function [A] = Q2A(q) % Quaternion to Euler angle
            eta = q(4);
            eps1 = q(1);
            eps2 = q(2);
            eps3 = q(3);

            eps = [eps1 eps2 eps3]';
            A = (eta^2- eps'*eps)*eye(3)+ 2*eps*eps'-2*eta*SKEW(eps); % R_B_O from O-->B
        end

        function [xi] = XI(quat) % QI matrix
            eps = quat(1:3);
            eta = quat(4);
            xi = [eta*eye(3)+SKEW(eps);-eps'];

            size(xi);
        end

        function [Q2] = NormalizeQ(Q1)  % Normalising quaternions
            rootQ1 = norm(Q1,2);
            Q2 = (1/rootQ1).*Q1;
        end

    end

    methods
        function initKalman(obj)
            % From kal_atti_init.m
            qk = [0; 0; 0; 1];  % q0 is R-P-Y, where 0 is the initial value
            biask = [0; 0; 0];
            obj.X_k = [qk; biask];

            sig_n = sqrt((obj.sig_es*pi/180)^2+(obj.sig_ss*pi/180)^2+(obj.sig_mag)^2);
            obj.P_k = obj.T_D^(1/4)*sig_n^(1/2)*(obj.sig_v^2+2*obj.sig_u*obj.sig_v*obj.T_D^(1/2))^(1/4)*eye(6);

            obj.w_k = obj.Gyro_m;
 end
        
        % From kal_atti.m
        function [Xerr] = runKalman(obj, flags)
            matrix_033 = zeros(3,3);
            bias_k = obj.X_k(5:7);
            bias_k1_k = bias_k;     % Bias prediction

            Skew_w = SKEW(obj.w_k);
            Mag_w = norm(obj.w_k);
            psi_k = (sin(1/2*Mag_w*obj.T_D)/Mag_w)*obj.w_k;
            Omega = [cos(1/2*Mag_w*obj.T_D)*eye(3)-SKEW(psi_k) psi_k;...
                -psi_k'     cos(1/2*Mag_w*obj.T_D)];

            q_k = obj.X_k(1:4);
            q_k1_k = Omega * q_k;

            Phi_11 = eye(3)- Skew_w*sin(Mag_w*obj.T_D)/Mag_w + Skew_w^2*(1-cos(Mag_w*obj.T_D))/Mag_w^2;
            Phi_12 = Skew_w*(1-cos(Mag_w*obj.T_D))/Mag_w^2-eye(3)*obj.T_D-Skew_w^2*(Mag_w*obj.T_D-sin(Mag_w*obj.T_D))/Mag_w^3;
            Phi_21 = matrix_033;
            Phi_22 = eye(3);

            Phi = [Phi_11 Phi_12; Phi_21 Phi_22];        % Siscretized Phi State transition matrix

            Gk = [-eye(3) zeros(3); zeros(3) eye(3)];    % Noise coefficient matrix
            Qk = [(obj.sig_v^2*obj.T_D+1/3*obj.sig_u^2*obj.T_D^3)*eye(3) -(1/2*obj.sig_u^2*obj.T_D^2)*eye(3);...
                -(1/2*obj.sig_u^2*obj.T_D^2)*eye(3)    (obj.sig_u^2*obj.T_D)*eye(3)];   % System noise variance

            P_k1_k = Phi * obj.P_k * Phi' + Gk * Qk * Gk';

            if isequal(flags, [0 0 0])    % No prediction value
                obj.Kalman = 0;
                P_k1 = P_k1_k;
                q_k1 = q_k1_k;
                bias_k1 = bias_k1_k;

                obj.X_k = [q_k1; bias_k1];
                obj.P_k = obj.P_k1;
                Xerr = zeros(6,1);
            else
                obj.Kalman = 1;
                Att = Q2A(q_k1_k);
                delX = zeros(6,1);
            end

            % Update magnetometer
            if (flags(1) == 1)
                H_B = [SKEW(Att*obj.Mag_I) matrix_033];
                R_B = obj.sig_mag^2*eye(3);
                res_B = obj.Mag_B - Att*obj.Mag_I;   % Measured value
            end

            % Update Sun sensor
            if (flags(2) == 1)
                H_S = [SKEW(Att*obj.Sun_I) matrix_033];
                R_S = obj.sig_ss^2*eye(3);
                res_S = obj.Sun_B - Att*obj.Sun_I;   % Estimated measurement in Body frame
            end

            % Update Earth sensor
            if (flags(3) == 1)
                H_E = [SKEW(Att*obj.Earth_I) matrix_033];
                R_E = obj.sig_es^2*eye(3);
                res_E = obj.Earth_B - Att*obj.Earth_I;
            end

            % Combination of sensors
            if isequal(flags, [0 0 1])
                H = H_E;      
                R = R_E;    
                res = res_E;

            elseif isequal(flags, [0 1 0])
                H = H_S;    
                R = R_S;       
                res = res_S;

            elseif isequal(flags, [0 1 1])
                H = H_E;      
                R = R_E;    
                res = res_E;

            elseif isequal(flags, [1 0 0])
                H = H_B;
                R = R_B;
                res = res_B;

            elseif isequal(flags, [1 0 1])
                H = [H_B;H_E ];
                R = [R_B matrix_033; matrix_033 R_E];
                res = [res_B;res_E];

            elseif isequal(flags, [1 1 0])
                H = [H_B;H_S];
                R = [R_B matrix_033; matrix_033 R_S ];
                res = [res_B;res_S];

            elseif isequal(flags, [1 1 1])
                H = [H_B; H_S; H_E];
                R = [R_B matrix_033 matrix_033;matrix_033 R_S matrix_033;matrix_033 matrix_033 R_E];
                res = [res_B;res_S;res_E];
            end

            % Update gain
            K = (P_k1_k * H')/(H*P_k1_k*H'+R);

            % Measurement update
            P_k1 = (eye(6)-K*H)*P_k1_k;
            delX = delX + K *(res-H*delX);
            Xerr = K*(res-H*delX);

            % Update for magnetometer measurement
            q_k1 = q_k1_k +1/2*XI(q_k1_k) * delX(1:3,1);
            q_k1 = NormalizeQ(q_k1);

            bias_k1 = bias_k1_k + delX(4:6,1);

            obj.w_k = obj.Gyro_m - bias_k1; % Gyro output - estimated bias

            % Save previous values
            obj.X_k = [q_k1; bias_k1];
            obj.P_k = P_k1;
        end

        % From atti_cal_cq.m
        function [Q_B_IB_Sins] = updateKalman(obj, Q_B_IB_Sins)
            q_k = Q_B_IB_Sins;
            obj.w_k = obj.Gyro_m - obj.bias;

            Mag_w = norm(obj.w_k);
            psi_k = (sin(1/2*Mag_w*obj.T_D)/Mag_w)*obj.w_k;
            Omega = [cos(1/2*Mag_w*obj.T_D)*eye(3)-SKEW(psi_k) psi_k;...
                -psi_k'     cos(1/2*Mag_w*obj.T_D)];

            q_k1_k = Omega * q_k;
            Q_B_IB_Sins = NormalizeQ(q_k1_k);
        end
    end
end