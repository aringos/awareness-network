clear all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameterizations go here. In the future, this should be a function 
% where these are passed in (so that we can loop it)

plotGeometry            = 1;
plotSensorEstimates     = 1;
plotNetworkPacketDelays = 1;

dt = 0.001;
tend = 5.0;
t = 0:dt:tend;
randomSeed = 0;

sensors = [Sensor('R20A', [30;60], 240*pi/180, 2)];
network = Network('WiFi');
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
       %Triangulate as necessary
   end

   %For each filter extrap in queue
       %If extrap is full rank, update to fusion center
    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Telemetry stuff

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


