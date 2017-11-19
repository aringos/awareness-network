clear all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameterizations go here. In the future, this should be a function 
% where these are passed in (so that we can loop it)

dt = 0.001;
tend = 5.0;
t = 0:dt:tend;

sensors = [Sensor('R20A', [30;60], 240*pi/180, 2)];
accel  = vehicleMotion( 'cruise', dt, tend );          


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Get target state for all time t
x = getStateVector(accel, dt);

%Simulation Loop
for k=1:length(t)
   currState = x(:,k);
    
   %Update sensors and their trackers
   for s=1:length(sensors)
       sensors(s) = sensors(s).update(currState,t(k));
   end
       
   %On communication rate boundary
       %Loop over sensors
           %If the sensor's kf is valid add filter extrap to queue
       %For each filter extrap in queue
           %If extrap is not full rank, loop over filter extraps again
           %(starting from this extrap)
               %If we found two angle-only extraps that arent same sensor,
                   %Triangulate and add that measurement to queue
       %For each filter extrap in queue
           %If extrap is full rank, update to fusion center
    
end


for s=1:length(sensors)
   sensors(s).plotTelemetry(); 
end

figure; hold on; grid on; axis equal;
plot(x(1,:), x(2,:));
for i=1:length(sensors)
   plot(sensors(i).vertices(1,:), sensors(i).vertices(2,:), 'Color', sensors(i).color); 
end

