clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameterizations go here. In the future, this should be a function 
% where these are passed in (so that we can loop it)

plotGeometry            = 1;
plotSensorEstimates     = 0;
plotNetworkPacketDelays = 0;
plotFusion              = 1;
plotTrueState           = 0;
plotEstimates           = 1;

dt = 0.001;
tend = 30.0;
t = 0:dt:tend;
randomSeed = 0;

sensors = [Sensor('Raspberry_Pi_Camera', [30;50],  235*pi/180, 2); ...
           Sensor('Raspberry_Pi_Camera', [-30;100], -55*pi/180, 2);
           Sensor('Raspberry_Pi_Camera', [30;150], 235*pi/180, 2);
           Sensor('Raspberry_Pi_Camera', [-30;200], -55*pi/180, 2);
           Sensor('Raspberry_Pi_Camera', [30;250], 235*pi/180, 2);
           Sensor('Raspberry_Pi_Camera', [-30;300], -55*pi/180, 2);
           Sensor('Raspberry_Pi_Camera', [30;350], 235*pi/180, 2);
           Sensor('Raspberry_Pi_Camera', [-30;400], -55*pi/180, 2)];
       
% sensors = [Sensor('Delphi_Mid_ESR', [20;60], 245*pi/180, 2); ...
%            Sensor('Delphi_Mid_ESR', [20;95], 245*pi/180, 2); ...
%            Sensor('Delphi_Mid_ESR', [20;130], 245*pi/180, 2); ...
%            Sensor('Delphi_Mid_ESR', [20;165], 245*pi/180, 2); ...
%            Sensor('Delphi_Mid_ESR', [20;200], 245*pi/180, 2); ...
%            Sensor('Delphi_Mid_ESR', [20;235], 245*pi/180, 2)];
       

       
network = Network('WiFi');
fusion  = FusionCenter();  
accel   = vehicleMotion( 'cruise', dt, tend );          

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rng(randomSeed);
x_hist = zeros(4,length(t));
P_hist = zeros(4,length(t));

%Get target state for all time t
x = getStateVector(accel, dt);

%Simulation Loop
for k=1:length(t)
   currState = x(:,k);
    
   %Update sensors and their trackers
   for s=1:length(sensors)
       sensors(s) = sensors(s).update(currState,t(k));
   end
       
   %Get observations (will be timestamped, not necessarily all aligned)
   rawObservationList = [];
   if network.observationsAvailable(t(k))
      [network, rawObservationList] = ...
          network.rawObservationListAtTime(sensors, t(k)); 
   end
   
   %Convert rawObservationList into refinedObservationList:
   %Basically, any necessary triangulation is performed here and we
   %assume that these will all have full rank (angle,range,rangerate)
   refinedObservationList = [];
   for i=1:length(rawObservationList)
       if rawObservationList(i).observation_H == [1;0;1;1]
          observation = FusionObservation( ...
                            rawObservationList(i).observation_z, ...
                            rawObservationList(i).observation_P, ...
                            rawObservationList(i).observation_t, ...
                            rawObservationList(i).sensor_pos);
          observation.type = 'RADAR';
          refinedObservationList = [refinedObservationList observation]; 
       elseif rawObservationList(i).observation_H == [1;1;0;0]
           for j=i+1:length(rawObservationList)
               if rawObservationList(i).observation_H == [1;1;0;0]
                  refinedObservationList = [refinedObservationList ...
                      triangulateObservations(rawObservationList(i), ...
                          rawObservationList(j))];
               end
           end
       end
   end

   %Update Sensor fusion kalman filter
   for i=1:length(refinedObservationList)
       fusion = fusion.update(refinedObservationList(i));
   end
    
   if plotEstimates
      [x_est P_est] = fusion.extrapolate(t(k));
      x_hist(:,k) = x_est;
      P_hist(:,k) = P_est;
   end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Telemetry stuff


if plotTrueState
   figure;
   subplot(4,1,1);
   plot(t, x(1,:)); title('X POS');
   subplot(4,1,2);
   plot(t, x(2,:)); title('Y POS');
   subplot(4,1,3);
   plot(t, x(3,:)); title('X VEL');
   subplot(4,1,4);
   plot(t, x(4,:)); title('Y VEL');
end

if plotNetworkPacketDelays
    network.plotPacketDelays();
end

if plotSensorEstimates
    for s=1:length(sensors)
       sensors(s).plotTelemetry(); 
    end
end

if plotGeometry
    figure; hold on; grid on; axis equal;
    plot(x(1,:), x(2,:));
    for i=1:length(sensors)
       plot(sensors(i).vertices(1,:), sensors(i).vertices(2,:), ...
           'Color', sensors(i).color); 
    end
end

if plotFusion
    fusion.plotTelemetry();
end

if plotEstimates
   figure;
   subplot(4,1,1); hold on; grid on; title('X Position');
   plot(t, x(1,:), 'g-', 'LineWidth', 2);
   plot(t, x_hist(1,:), 'k-');
   plot(t, x_hist(1,:)+sqrt(P_hist(1,:)), 'r--');
   plot(t, x_hist(1,:)-sqrt(P_hist(1,:)), 'r--');
   subplot(4,1,2); hold on; grid on; title('X Velocity');
   plot(t, x(3,:), 'g-', 'LineWidth', 2);
   plot(t, x_hist(2,:), 'k-');
   plot(t, x_hist(2,:)+sqrt(P_hist(2,:)), 'r--');
   plot(t, x_hist(2,:)-sqrt(P_hist(2,:)), 'r--');
   subplot(4,1,3); hold on; grid on; title('Y Position');
   plot(t, x(2,:), 'g-', 'LineWidth', 2);
   plot(t, x_hist(3,:), 'k-');
   plot(t, x_hist(3,:)+sqrt(P_hist(3,:)), 'r--');
   plot(t, x_hist(3,:)-sqrt(P_hist(3,:)), 'r--');
   subplot(4,1,4); hold on; grid on; title('Y Velocity');
   plot(t, x(4,:), 'g-', 'LineWidth', 2);
   plot(t, x_hist(4,:), 'k-');
   plot(t, x_hist(4,:)+sqrt(P_hist(4,:)), 'r--');
   plot(t, x_hist(4,:)-sqrt(P_hist(4,:)), 'r--');
end

