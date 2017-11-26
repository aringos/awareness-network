classdef Network
    
    properties
        communicationRatebps     = 0; 
        communicationRangeMeters = 0;
        lastPacketReceivedTime   = -10.0;
        packetDt                 = 0.03;
        obsDelay_t               = [];
        obsDelay_hist            = [];
    end
    
    methods
        
        function network = Network(protocolName)
            switch(protocolName)
                case 'WiFi'
                    network.communicationRatebps     = 1e6;
                    network.communicationRangeMeters = 50;
                case 'LoRa'
                    network.communicationRatebps     = 300e3;
                    network.communicationRangeMeters = 5000;
                case 'Zigbee_LowPower'
                    network.communicationRatebps     = 256e3;
                    network.communicationRangeMeters = 60;
                case 'Zigbee_HighPower'
                    network.communicationRatebps     = 256e3;
                    network.communicationRangeMeters = 90;
            end
        end
        
        function available = observationsAvailable(network, t)
           available = (t-network.lastPacketReceivedTime)>=network.packetDt;
        end
        
        function [network, obsList] = rawObservationListAtTime(network, sensors, t)
            obsList = [];
            for s=1:length(sensors)
               if sensors(s).tracking
                   obsTime = t-sensors(s).dt*rand()*0.5;
                   obsList = [obsList SensorDataPacket(sensors(s), obsTime)];
                   network.obsDelay_t = [network.obsDelay_t t];
                   network.obsDelay_hist = [network.obsDelay_hist t-obsTime];
                   network.packetDt = sensors(s).dt;
               end
            end
            network.lastPacketReceivedTime = t;
        end
        
        function plotPacketDelays(network)
            figure; hold on; grid on; title('Observation Packet delays');
            scatter(network.obsDelay_t, network.obsDelay_hist, 'bX');
            xlabel('Time received (t)'); ylabel('Observation delay (t-obsTime)');
        end
    end
    
end

