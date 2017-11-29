classdef scn_RaspPi1080p12M12_LoRa
    
    properties
        sensors = [];
        network = 0;
        hardware = 0;
        fusion = 0;
        vehicleAccel = [];
    end
    
    methods

        function scenario = scn_RaspPi1080p12M12_LoRa(dt, tend)
            scenario.sensors = ...
              [Sensor('Pi_12mmM12', [-20;275], 277*pi/180); 
               Sensor('Pi_12mmM12', [20;375],  -97*pi/180);
               Sensor('Pi_12mmM12', [-20;475], 277*pi/180);
               Sensor('Pi_12mmM12', [20;575], -97*pi/180);
               Sensor('Pi_12mmM12', [-20;675], 277*pi/180)];
            
            scenario.network      = Network('LoRa_RN2483A');
            scenario.hardware     = Hardware('Pi_3');
            scenario.fusion       = FusionCenter();  
            scenario.vehicleAccel = vehicleMotion( 'cruise', dt, tend );    
        end
     
    end
    
end

