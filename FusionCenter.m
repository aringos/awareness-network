classdef FusionCenter
    
    properties
        x = zeros(4,1);
        P = eye(4,4);
        Q = 0.5*eye(4,4);
        t = 0.0;
        
        P_diag_hist = zeros(4,1);
        x_hist      = zeros(4,1);
        t_hist      = [0];
        
        isInitialized = 0;
    end
    
    methods
        
        function [H,x_pred,z_pred] = getPredictionsForSensor(ekf, sensorPos)
           relPos = [ekf.x(1)-sensorPos(1); ekf.x(3)-sensorPos(2)];
           relVel = [ekf.x(2); ekf.x(4)];
           x_pred = [relPos(1); relVel(1); relPos(2); relVel(2)];
           xpos   = relPos(1);
           xdot   = relVel(1);
           ypos   = relPos(2);
           ydot   = relVel(2);
           R      = norm(relPos);
           Rdot   = dot(relPos,relVel)/R;
           
           dTheta_dx   = -ypos/(R^2);
           dTheta_dy   = xpos/(R^2);
           dR_dx       = xpos/R;
           dR_dy       = ypos/R;
           dRdot_dx    = (xdot*R-xpos*Rdot)/R^2;
           dRdot_dxdot = xpos/R;
           dRdot_dy    = (ydot*R-ypos*Rdot)/R^2;
           dRdot_dydot = ypos/R;
           
           H = [dTheta_dx 0           dTheta_dy  0; ...
                dR_dx     0           dR_dy      0; ...
                dRdot_dx  dRdot_dxdot dRdot_dy   dRdot_dydot];
           z_pred = (H*x_pred)'; 
           z_pred(1) = atan2(x_pred(3),x_pred(1));
        end
        
        function phi = getPhi(ekf, dt)
           phi = [1  dt 0  0; ...
                  0  1  0  0; ...
                  0  0  1  dt; ...
                  0  0  0  1  ];
        end
        
        function ekf = setProcessNoise(ekf, dt)
           dt2   = dt*dt;
           dt3   = dt2*dt;
           ekf.Q = [dt3/3 dt2/2 0     0; ...
                    dt2/2 dt    0     0; ...
                    0     0     dt3/3 dt2/2; ...
                    0     0     dt2/2 dt];
           ekf.Q = 5e-2*eye(4,4); %50*ekf.Q;
        end
        
        function ekf = measUpdate(ekf, z, R, t, sensorPos)
           [H, x_pred, z_pred] = ekf.getPredictionsForSensor(sensorPos);
           R           = diag(R);
           dt          = t-ekf.t;
           ekf         = ekf.setProcessNoise(dt);
           phi         = ekf.getPhi(dt);
           z_residual  = z'-z_pred;
           K           = phi*ekf.P*H'*inv(H*ekf.P*H'+R);
           ekf.x       = phi*x_pred + K*z_residual';
           ekf.x       = ekf.x + [sensorPos(1); 0; sensorPos(2); 0];
           ekf.P       = (phi-K*H)*ekf.P*phi'+ekf.Q;
           ekf.t       = t;
        end
        
        function ekf = initialize(ekf, z, R, t, sensorPos)
           azDot0   = 0.01;
           ekf.x(1) = sensorPos(1) + z(2)*cos(z(1));
           ekf.x(2) = sensorPos(2) + z(2)*sin(z(1));
           ekf.x(3) = z(3)*cos(z(1))-z(2)*sin(z(1))*azDot0;
           ekf.x(4) = z(3)*sin(z(1))+z(2)*cos(z(1))*azDot0;
           ekf.P(1,1) = R(2)*cos(R(1));
           ekf.P(2,2) = R(2)*sin(R(1));
           ekf.P(3,3) = R(3)*cos(R(1))-R(2)*sin(R(1))*azDot0;
           ekf.P(4,4) = R(3)*sin(R(1))+R(2)*cos(R(1))*azDot0;       
           ekf.t    = t;
           ekf.isInitialized = 1;
        end
        
        function ekf = update(ekf, observationPacket)
           observationPacket = observationPacket.convertToRadarObservations();
           obsZ   = observationPacket.observation_z;
           obsR   = eye(3,3)*observationPacket.observation_P;
           obsT   = observationPacket.observation_t;
           if ekf.isInitialized
              ekf = ekf.measUpdate(obsZ,obsR,obsT,observationPacket.sensor_pos);
           else 
              ekf = ekf.initialize(obsZ,obsR,obsT,observationPacket.sensor_pos);
           end
           ekf = ekf.recordTelemetry();
        end
        
        function ekf = recordTelemetry(ekf)
           ekf.P_diag_hist = [ekf.P_diag_hist diag(ekf.P)];
           ekf.x_hist      = [ekf.x_hist      ekf.x];
           ekf.t_hist      = [ekf.t_hist      ekf.t];       
        end
        
        function ekf = plotTelemetry(ekf)
           figure;
           subplot(4,1,1); hold on; grid on;
           title('X Position');
           plot(ekf.t_hist, ekf.x_hist(1,:));
           plot(ekf.t_hist, ekf.x_hist(1,:)+ekf.P_diag_hist(1,:)*3, 'r--');
           plot(ekf.t_hist, ekf.x_hist(1,:)-ekf.P_diag_hist(1,:)*3, 'r--');
           subplot(4,1,2); hold on; grid on;
           title('X Velocity');
           plot(ekf.t_hist, ekf.x_hist(2,:));
           plot(ekf.t_hist, ekf.x_hist(2,:)+ekf.P_diag_hist(2,:)*3, 'r--');
           plot(ekf.t_hist, ekf.x_hist(2,:)-ekf.P_diag_hist(2,:)*3, 'r--');
           subplot(4,1,3); hold on; grid on;
           title('Y Position');
           plot(ekf.t_hist, ekf.x_hist(3,:));
           plot(ekf.t_hist, ekf.x_hist(3,:)+ekf.P_diag_hist(3,:)*3, 'r--');
           plot(ekf.t_hist, ekf.x_hist(3,:)-ekf.P_diag_hist(3,:)*3, 'r--');
           subplot(4,1,4); hold on; grid on;
           title('Y Velocity');
           plot(ekf.t_hist, ekf.x_hist(4,:));
           plot(ekf.t_hist, ekf.x_hist(4,:)+ekf.P_diag_hist(4,:)*3, 'r--');
           plot(ekf.t_hist, ekf.x_hist(4,:)-ekf.P_diag_hist(4,:)*3, 'r--');
        end
        
    end
    
end

