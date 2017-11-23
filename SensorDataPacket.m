classdef SensorDataPacket
% This class represents the data sent from a single sensor.
% The actual network packets are aggregations of N Sensor packets
% based on Sensor Packet size. The network calculates how large N
% can be based on datarate, which determines range of the network.
    
    properties
        totalDataSizeBits         = 0;
        
        observation_t             = 0;
        observation_z             = 0;
        observation_P             = 0;
        observation_H             = [];
        
        sensor_pos                = [];
    end
    
    methods
        function packet = SensorDataPacket(sensor, t)
           packet.observation_t     = t; 
           packet.totalDataSizeBits = sensor.P_sizeBits + ...
                                      sensor.z_sizeBits + ...
                                      sensor.id_sizeBits + ...
                                      sensor.tgtClass_sizeBits + ...
                                      sensor.obsTime_sizeBits;
           packet.totalDataSizeBits = packet.totalDataSizeBits * ...
                                      sensor.maxNumObjectsTracked;
                                 
           [packet.observation_z, packet.observation_P] = ...
               sensor.filter.getExtrapolation(t);
           packet.observation_H = sensor.H;
           packet.sensor_pos  = sensor.pos;
        end
        
        function packet = convertToRadarObservations(packet)
            packet.observation_z = [packet.observation_z(1); ...
                                    packet.observation_z(3); ...
                                    packet.observation_z(4)];
            packet.observation_P = [packet.observation_P(1,1); ...
                                    packet.observation_P(3,3); ...
                                    packet.observation_P(4,4)];           
        end
    end
    
end

