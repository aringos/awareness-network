classdef CostPerformanceEstimate

    properties
        networkValid      = 0;
        x_residuals       = 0;
        x_errorMean       = 0;
        x_errorStdev      = 0;
        networkDistMeters = 0;
        costPerMeter      = 0;
        costPerMile       = 0;
        wattsPerMeter      = 0;
        wattsPerMile       = 0;
        adjacencies       = [];
    end
    
    methods
        
        %Build undirected graph adjacency matrix and check reachability
        %For all nodes to verify if network structure is valid
        function est = validate(est, sensors, network)
            est.networkValid = 1;
            est.adjacencies = zeros(length(sensors), length(sensors));
            for i=1:length(sensors)
                for j=1:length(sensors)
                   commDist = norm(sensors(i).pos-sensors(j).pos);
                   if commDist<network.communicationRangeMeters && (i~=j)
                       est.adjacencies(i,j)=commDist;
                   else
                       est.adjacencies(i,j)=0;
                   end
                end
            end
            netGraph = graph(est.adjacencies~=0);
            for i=1:length(sensors)
               for j=i:length(sensors)
                  if i==j 
                      continue;
                  end
                  pathTree = shortestpathtree(netGraph, i, j);
                  if height(pathTree.Edges)==0
                     disp(sprintf('INVALID NETWORK: No route from sensor %d to %d', i, j));
                     est.networkValid = 0;
                  end
               end
            end
            if ~est.networkValid
               disp('Network does not support this layout. See adjacency list for valid links.');
               est.adjacencies
            end
        end
        
        function est = calcErrors(est, x, x_hist)
            xpos_err            = x_hist(1,:)-x(1,:);
            ypos_err            = x_hist(3,:)-x(2,:);
            xvel_err            = x_hist(2,:)-x(3,:);
            yvel_err            = x_hist(4,:)-x(4,:);
            est.x_residuals     = [xpos_err; xvel_err; ypos_err; yvel_err];
            est.x_errorMean     = [mean(abs(est.x_residuals(1,:)));
                                   mean(abs(est.x_residuals(2,:)));
                                   mean(abs(est.x_residuals(3,:)));
                                   mean(abs(est.x_residuals(4,:)))];
            est.x_errorStdev    = [std(est.x_residuals(1,:));
                                   std(est.x_residuals(2,:));
                                   std(est.x_residuals(3,:));
                                   std(est.x_residuals(4,:))];
        end
        
        function est = calcNetworkDistance(est, sensors, network)
            minDist = min(est.adjacencies(find(est.adjacencies~=0)));
            packetSize = SensorDataPacket(sensors(1), 1.0).totalDataSizeBits;
            disp(sprintf('Assuming average hop of %d meters', minDist));
            disp(sprintf('Sensor updates every %f sec', sensors(1).dt));
            disp(sprintf('Sensor data packet size %d bits', packetSize));
            packetsPerSecPerSensor = (1.0/sensors(1).dt);
            bitsPerPacketPerSensor = packetSize;
            bitsPerSecondPerSensor = bitsPerPacketPerSensor*packetsPerSecPerSensor;
            disp(sprintf('Single node produces %d bits per sec', bitsPerSecondPerSensor));
            numSensorsSupported = network.communicationRatebps / bitsPerSecondPerSensor;
            disp(sprintf('Network data rate supports %d sensor hops at full rate', numSensorsSupported));
            rangeCommunicationLimited = network.communicationRangeMeters * numSensorsSupported;
            est.networkDistMeters   = minDist * numSensorsSupported;
            fprintf('This network supports %f linear meters of observability.\n', est.networkDistMeters);
        end
        
        function est = calcCosts(est, x, sensors, network, hardware)
            disp('Note: Cost calculations are assuming entire trajectory is observed!');
            totalDisplacementMeters = norm([x(1,end)-x(1,1); x(2,end)-x(2,1)]);
            costPerUnit             = sensors(1).costPerUnit + ...
                                      network.costPerUnit + ...
                                      hardware.costPerUnit + ...
                                      hardware.miscCosts;
            fprintf('Estimated total cost per unit: %.2f\n', costPerUnit);
            est.costPerMeter = costPerUnit*length(sensors)/totalDisplacementMeters;
            est.costPerMile  = est.costPerMeter*1609.34;
            fprintf('Cost/distance: %.2f $/meter, %.2f $/mile\n', est.costPerMeter, est.costPerMile);
            wattsPerUnit = sensors(1).powerDrawPerUnit_W + ...
                           network.powerDrawPerUnit_W + ...
                           hardware.powerDrawPerUnit_W;
            est.wattsPerMeter = wattsPerUnit*length(sensors)/totalDisplacementMeters;
            est.wattsPerMile = est.wattsPerMeter*1609.34;
            fprintf('Power/distance: %.2f W/meter, %.2f W/mile\n', est.wattsPerMeter, est.wattsPerMile);
        end
        
        function plotResiduals(est, t)
           figure;
           subplot(4,1,1); hold on; grid on; title('X Position Estimate Error');
           p1 = plot(t, abs(est.x_residuals(1,:)), 'k-');
           p2 = plot(t, est.x_errorMean(1)*ones(size(t)), 'b--');
           p3 = plot(t, est.x_errorStdev(1)*ones(size(t)), 'r--');
           legend([p1, p2, p3], 'Estimate error', 'Error Mean', 'Error stdev');
           axis([-inf inf 0 2.0*est.x_errorStdev(1)]);
           subplot(4,1,2); hold on; grid on; title('X Velocity Estimate Error');
           p1 = plot(t, abs(est.x_residuals(2,:)), 'k-');
           p2 = plot(t, est.x_errorMean(2)*ones(size(t)), 'b--');
           p3 = plot(t, est.x_errorStdev(2)*ones(size(t)), 'r--');
           legend([p1, p2, p3], 'Estimate error', 'Error Mean', 'Error stdev');
           axis([-inf inf 0 2.0*est.x_errorStdev(2)]);
           subplot(4,1,3); hold on; grid on; title('Y Position Estimate Error');
           p1 = plot(t, abs(est.x_residuals(3,:)), 'k-');
           p2 = plot(t, est.x_errorMean(3)*ones(size(t)), 'b--');
           p3 = plot(t, est.x_errorStdev(3)*ones(size(t)), 'r--');
           legend([p1, p2, p3], 'Estimate error', 'Error Mean', 'Error stdev');
           axis([-inf inf 0 2.0*est.x_errorStdev(3)]);
           subplot(4,1,4); hold on; grid on; title('Y Velocity Estimate Error');
           p1 = plot(t, abs(est.x_residuals(4,:)), 'k-');
           p2 = plot(t, est.x_errorMean(4)*ones(size(t)), 'b--');
           p3 = plot(t, est.x_errorStdev(4)*ones(size(t)), 'r--');
           legend([p1, p2, p3], 'Estimate error', 'Error Mean', 'Error stdev');
           axis([-inf inf 0 2.0*est.x_errorStdev(4)]);
        end
        
        function est = CostPerformanceEstimate(x, x_hist, sensors, network, hardware)
            est = est.validate(sensors, network);
            est = est.calcErrors(x, x_hist);
            est = est.calcNetworkDistance(sensors, network);
            est = est.calcCosts(x, sensors, network, hardware);
        end
    end
    
end

