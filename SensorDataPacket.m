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
        end
    end
    
end

