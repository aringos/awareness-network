%--------------------------------------------------------------------------
%
% simDriver
%
% Anthony Rodriguez
% Austin Smith
% SIE-554A
% 11/18/2017
%
% Runs the top-level simulation. Allows DOE studies over user-specified
% scenario sets stored in scenario classes.
%
% Inputs:  Plot toggles
%          Time vector
%          Scenario set stored in cell array
%
% Outputs: FOMs (cost vs. performance)
%          state estimate history (ENU frame)
%          error covariance history (ENU frame)
%          true states (ENU frame)
%
%--------------------------------------------------------------------------

clc;
addpath('scenarios');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Driver Setup

plotGeometry            = 1;
plotSensorEstimates     = 0;
plotNetworkPacketDelays = 0;
plotFusion              = 0;
plotTrueState           = 0;
plotEstimateResiduals   = 0;
plotEstimates           = 1;

dt         = 0.001;
tend       = 51.5; %A major block in midtown Tucson is about 600-800m long
                   %tend = 800m/15.6mps
randomSeed = 0;  

scenarioSet = ...
    { scn_RaspPi1080p_LoRa(dt,tend),...
      scn_DelphiLR_LoRa(dt,tend),...
      scn_DelphiMR_LoRa(dt,tend)};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Run Scenario Set

numScn = length(scenarioSet);
scnData = struct('metricEstimate',cell(1,numScn),'x_hist',cell(1,numScn),...
                 'P_hist',cell(1,numScn),'x',cell(1,numScn));
t = 0:dt:tend;
rng(randomSeed);

% Scenario loop
for scn = 1:numScn
    
    sensors  = scenarioSet{scn}.sensors;
    network  = scenarioSet{scn}.network;
    hardware = scenarioSet{scn}.hardware;
    fusion   = scenarioSet{scn}.fusion;
    accel    = scenarioSet{scn}.vehicleAccel;
    
    x_hist = zeros(4,length(t));
    P_hist = zeros(4,length(t));

    %Get target state for all time t
    x = getStateVector(accel,scenarioSet{scn}.vehicleInitD,...
                       scenarioSet{scn}.vehicleInitV,dt);

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

       [x_est,P_est] = fusion.extrapolate(t(k));
       x_hist(:,k) = x_est;
       P_hist(:,k) = P_est;
       
    end

    metricEstimate = CostPerformanceEstimate(x, x_hist, sensors, network, hardware);

    % Store scenario output
    scnData.metricEstimate{scn} = metricEstimate;
    scnData.x_hist{scn} = x_hist;
    scnData.P_hist{scn} = P_hist;
    scnData.x{scn} = x;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Telemetry Plots

    if plotTrueState
       figure;
       subplot(4,1,1);
       plot(t, x(1,:)); title('EAST POS');
       subplot(4,1,2);
       plot(t, x(2,:)); title('NORTH POS');
       subplot(4,1,3);
       plot(t, x(3,:)); title('EAST VEL');
       subplot(4,1,4);
       plot(t, x(4,:)); title('NORTH VEL');
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
        title('Scenario Geometry');
        xlabel('East (m)'); ylabel('North (m)');
        plot(x(1,:), x(2,:), 'k-', 'LineWidth', 2);
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
       subplot(4,1,1); hold on; grid on; title('East Position (m)');
       plot(t, x(1,:), 'g-', 'LineWidth', 2);
       plot(t, x_hist(1,:), 'k-');
       plot(t, x_hist(1,:)+sqrt(P_hist(1,:)), 'r--');
       plot(t, x_hist(1,:)-sqrt(P_hist(1,:)), 'r--');
       subplot(4,1,2); hold on; grid on; title('East Velocity (m/s)');
       plot(t, x(3,:), 'g-', 'LineWidth', 2);
       plot(t, x_hist(2,:), 'k-');
       plot(t, x_hist(2,:)+sqrt(P_hist(2,:)), 'r--');
       plot(t, x_hist(2,:)-sqrt(P_hist(2,:)), 'r--');
       subplot(4,1,3); hold on; grid on; title('North Position (m)');
       plot(t, x(2,:), 'g-', 'LineWidth', 2);
       plot(t, x_hist(3,:), 'k-');
       plot(t, x_hist(3,:)+sqrt(P_hist(3,:)), 'r--');
       plot(t, x_hist(3,:)-sqrt(P_hist(3,:)), 'r--');
       subplot(4,1,4); hold on; grid on; title('North Velocity (m/s)');
       plot(t, x(4,:), 'g-', 'LineWidth', 2);
       plot(t, x_hist(4,:), 'k-');
       plot(t, x_hist(4,:)+sqrt(P_hist(4,:)), 'r--');
       plot(t, x_hist(4,:)-sqrt(P_hist(4,:)), 'r--');
       xlabel('Time (s)');
    end

    if plotEstimateResiduals
       metricEstimate.plotResiduals(t); 
    end

end
