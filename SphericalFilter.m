classdef SphericalFilter
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x             = [0;0;0;0];
        z             = [0;0;0;0];
        P             = zeros(3,3);
        t             = -1.0;
        phi           = zeros(4,4);
        Q             = [1e-2 0 0 0; ...
                         0 1e-2 0 0; ...
                         0 0 1e-2 0; ...
                         0 0 0 1e-2];
        H             = eye(4,4);
        measH         = zeros(4,1);
        trap_azRate   = 0; 
        x_hist        = [0;0;0;0];
        z_hist        = [0;0;0;0];
        P_diag_hist   = [0;0;0;0];
        residual_hist = [0;0;0;0];
        t_hist        = [0];
    end
    
    methods
        function filter = initialize(filter, z, R, H, t)
            filter.t = t;
            filter.x = z;
            filter.P = R;
            filter.measH = H;
            filter = filter.recordTelemetry(z,[0;0;0;0],t);
        end
        
        function [z,P] = getExtrapolation(filter, t)
            dt     = t - filter.t;         
            filter.phi = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];
            z      = filter.phi*filter.x;
            P      = filter.phi*filter.P*filter.P'+filter.Q;            
        end
        
        function filter = update(filter, z, R, t)
            dt = t - filter.t;     
            filter.phi = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];
            x_predicted = filter.phi*filter.x;
            P_predicted = filter.phi*filter.P*filter.P'+filter.Q;
            residual    = z - filter.H*x_predicted;
            innovation  = R + filter.H*P_predicted*filter.H';
            kalmanGain  = P_predicted*filter.H'*inv(innovation);
            filter.x    = x_predicted + kalmanGain*residual;
            filter.P    = P_predicted - kalmanGain*innovation*kalmanGain';
            filter = filter.recordTelemetry(z,residual,t);
            filter.t = t;
        end
        
        function filter = recordTelemetry(filter, z, residual, t)
            filter.x_hist        = [filter.x_hist filter.x];
            filter.z_hist        = [filter.z_hist z];
            filter.residual_hist = [filter.residual_hist residual];
            filter.t_hist        = [filter.t_hist t];
            filter.P_diag_hist = [filter.P(1,1);...
                                  filter.P(2,2);...
                                  filter.P(3,3);
                                  filter.P(4,4)];            
        end
        
        function plotTelemetry(filter)
           figure; 
           subplot(4,1,1); grid on; hold on; title('Azimuth');
           plot(filter.t_hist, filter.x_hist(1,:).*180/pi, 'k-');
           plot(filter.t_hist, filter.x_hist(1,:).*180/pi+sqrt(filter.P_diag_hist(1,:).*180/pi)*3, 'r--');
           plot(filter.t_hist, filter.x_hist(1,:).*180/pi-sqrt(filter.P_diag_hist(1,:).*180/pi)*3, 'r--');
           scatter(filter.t_hist, filter.z_hist(1,:).*180/pi, 'kO');
           subplot(4,1,2); grid on; hold on; title('Azimuth Rate');
           plot(filter.t_hist, filter.x_hist(2,:).*180/pi, 'k-');
           plot(filter.t_hist, filter.x_hist(2,:).*180/pi+sqrt(filter.P_diag_hist(2,:).*180/pi)*3, 'r--');
           plot(filter.t_hist, filter.x_hist(2,:).*180/pi-sqrt(filter.P_diag_hist(2,:).*180/pi)*3, 'r--');
           scatter(filter.t_hist, filter.z_hist(2,:).*180/pi, 'kO');
           subplot(4,1,3); grid on; hold on; title('Range');
           plot(filter.t_hist, filter.x_hist(3,:), 'k-');
           plot(filter.t_hist, filter.x_hist(3,:)+sqrt(filter.P_diag_hist(3,:))*3, 'r--');
           plot(filter.t_hist, filter.x_hist(3,:)-sqrt(filter.P_diag_hist(3,:))*3, 'r--');
           scatter(filter.t_hist, filter.z_hist(3,:), 'kO');
           subplot(4,1,4); grid on; hold on; title('Range Rate');
           plot(filter.t_hist, filter.x_hist(4,:), 'k-');
           plot(filter.t_hist, filter.x_hist(4,:)+sqrt(filter.P_diag_hist(4,:))*3, 'r--');
           plot(filter.t_hist, filter.x_hist(4,:)-sqrt(filter.P_diag_hist(4,:))*3, 'r--');
           scatter(filter.t_hist, filter.z_hist(4,:), 'kO');
        end
        
    end
    
end

