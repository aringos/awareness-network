clear all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameterizations go here. In the future, this should be a function 
% where these are passed in (so that we can loop it)

plotGeometry            = 1;
plotSensorEstimates     = 1;
plotNetworkPacketDelays = 0;
plotFusion              = 1;
plotTrueState           = 1;

dt = 0.001;
tend = 20.0;
t = 0:dt:tend;
randomSeed = 0;

sensors = [Sensor('Delphi_Mid_ESR', [20;60], 245*pi/180, 2); ...
           Sensor('Delphi_Mid_ESR', [20;95], 245*pi/180, 2); ...
           Sensor('Delphi_Mid_ESR', [20;130], 245*pi/180, 2); ...
           Sensor('Delphi_Mid_ESR', [20;165], 245*pi/180, 2)];
network = Network('WiFi');
fusion  = FusionCenter();  
accel   = vehicleMotion( 'cruise', dt, tend );          

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rng(randomSeed);

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
          refinedObservationList = ...
              [refinedObservationList rawObservationList(i)]; 
       end
   end

   %Update Sensor fusion kalman filter
   for i=1:length(refinedObservationList)
       fusion = fusion.update(refinedObservationList(i));
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

