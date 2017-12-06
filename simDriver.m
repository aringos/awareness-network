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
%          Monte Carlo seed vector
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
plotEstimates           = 0;
plotMetrics             = 1;

dt         = 0.001;
tend       = 51.5; %A major block in midtown Tucson is about 600-800m long
                   %tend = 800m/15.6mps

%WARNING: Repeatability issues at present. Taking results from seed 1.
randomSeeds = 1;  

% Metric plot labels. Should add these to the sensor models.
sensorUnitNames = {'Raspberry Pi 1080p','Raspberry Pi 720p',...
                       'Arducam OV7670','Arducam OV5647'};
scenarioSet = ...
    { scn_RaspPi1080p_LoRa(dt,tend),...
      scn_RaspPi720p_LoRa(dt,tend),...
      scn_OV7670_LoRa(dt,tend),...
      scn_OV5647_LoRa(dt,tend)...
      %scn_DelphiLR_LoRa(dt,tend),...
      %scn_DelphiMR_LoRa(dt,tend)
      };

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Run Scenario Set

numScn = length(scenarioSet);
numSeeds = length(randomSeeds);
emptyCell = cell(numScn,numSeeds);
scnData = struct('metricEstimate',emptyCell,'x_hist',emptyCell,...
                 'P_hist',emptyCell,'x',emptyCell);
t = 0:dt:tend;

% Scenario Loop
for scn = 1:numScn
    
    sensors  = scenarioSet{scn}.sensors;
    network  = scenarioSet{scn}.network;
    hardware = scenarioSet{scn}.hardware;
    fusion   = scenarioSet{scn}.fusion;
    accel    = scenarioSet{scn}.vehicleAccel;

    %Get target state for all time t
    x = getStateVector(accel,scenarioSet{scn}.vehicleInitD,...
                       scenarioSet{scn}.vehicleInitV,dt);

    %Monte Carlo Loop
    for seed = 1:numSeeds
        rng(randomSeeds(seed));
        
        x_hist = zeros(4,length(t));
        P_hist = zeros(4,length(t));
        
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
        scnData(scn,seed).metricEstimate = metricEstimate;
        scnData(scn,seed).x_hist = x_hist;
        scnData(scn,seed).P_hist = P_hist;
        scnData(scn,seed).x = x;

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
end

% Plot metrics from all runs
if plotMetrics
    
    zeroVec  = zeros(numScn,1);
    costs    = zeroVec;
    power    = zeroVec;
    posBiases= zeroVec;
    posSig   = zeroVec;
    velBiases= zeroVec;
    velSig   = zeroVec;
    for scn = 1:numScn
       costs(scn)    = scnData(scn,1).metricEstimate.costPerMile;
       power(scn)    = scnData(scn,1).metricEstimate.wattsPerMile;
       posBiases(scn)= max(scnData(scn,1).metricEstimate.x_errorMean([1,3]));
       posSig(scn)   = max(scnData(scn,1).metricEstimate.x_errorStdev([1,3]));
       velBiases(scn)= max(scnData(scn,1).metricEstimate.x_errorMean([2,4]));
       velSig(scn)   = max(scnData(scn,1).metricEstimate.x_errorStdev([2,4]));
    end
    
    figure;
    bar(costs); box on; grid on;
    title('Cost Density ($/mile)','fontweight','bold','fontsize',26);
    set(gca,'xticklabel',sensorUnitNames,'fontsize',24);
    figure;
    bar(power); box on; grid on;
    title('Power Density (W/mile)','fontweight','bold','fontsize',26);
    set(gca,'xticklabel',sensorUnitNames,'fontsize',24);
    figure;
    bar(posBiases); box on; grid on;
    title('Mean Position Bias (m)','fontweight','bold','fontsize',26);
    set(gca,'xticklabel',sensorUnitNames,'fontsize',24);
    figure;
    bar(posSig); box on; grid on;
    title('Position Uncertainty, 1-\sigma (m)','fontweight','bold','fontsize',26);
    set(gca,'xticklabel',sensorUnitNames,'fontsize',24);
    figure;
    bar(velBiases); box on; grid on;
    title('Mean Velocity Bias (m/s)','fontweight','bold','fontsize',26);
    set(gca,'xticklabel',sensorUnitNames,'fontsize',24);
    figure;
    bar(velSig); box on; grid on;
    title('Velocity Uncertainty, 1-\sigma (m/s)','fontweight','bold','fontsize',26);
    set(gca,'xticklabel',sensorUnitNames,'fontsize',24);
    
    figure;
    onesVec = ones(numScn,1);
    costFigure = min(costs)./costs;
    powerFigure = min(power)./power;
    posBiasFigure = min(posBiases)./posBiases;
    posSigFigure = min(posSig)./posSig;
    bar([costFigure';powerFigure';posBiasFigure';posSigFigure']);
    box on; grid on;
    set(gca,'xticklabel',{'Cost Density','Power Density',...
        'Mean Position Bias','Position Uncertainty'});
    set(gca,'fontsize',24);
    title('Normalized Figures of Merit','fontweight','bold','fontsize',26);
    legend(sensorUnitNames);
    
    figure;
    costRss = sqrt(costFigure.^2+powerFigure.^2);
    perfRss = sqrt(posBiasFigure.^2+posSigFigure.^2);
    scatter(costRss,perfRss,16^2,linspace(1,10,numScn),'filled');
    box on; grid on;
    text(costRss+ones(numScn,1)*0.02,perfRss,sensorUnitNames,'fontsize',16);
    title('Cost vs. Performance Pareto','fontweight','bold','fontsize',26);
    set(gca,'fontsize',24);
    xlabel('Cost Score','fontsize',24); ylabel('Performance Score','fontsize',24);

end
