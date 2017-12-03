classdef scn_RaspPi1080p12M12_LoRa
    
    properties
        sensors = [];
        network = 0;
        hardware = 0;
        fusion = 0;
        vehicleAccel = [];
        vehicleInitD = zeros(2,1);
        vehicleInitV = zeros(2,1);
    end
    
    methods

        function scenario = scn_RaspPi1080p12M12_LoRa(dt, tend)
            scenario.sensors = ...
              [Sensor('Pi_12mmM12', [-20;275], 277*pi/180); 
               Sensor('Pi_12mmM12', [20;375],  -97*pi/180);
               Sensor('Pi_12mmM12', [-20;475], 277*pi/180);
               Sensor('Pi_12mmM12', [20;575],  -97*pi/180);
               Sensor('Pi_12mmM12', [-20;675], 277*pi/180);
               Sensor('Pi_12mmM12', [20;775],  -97*pi/180);
               Sensor('Pi_12mmM12', [-20;875], 277*pi/180)];
            
            scenario.network      = Network('LoRa_RN2483A');
            scenario.hardware     = Hardware('Pi_3');
            scenario.fusion       = FusionCenter();  
            
            mph2mps = unitsratio('m','mi')/3600;
            scenario.vehicleAccel = vehicleMotion( 'cruise', dt, tend ); 
            scenario.vehicleInitD = [0;0];
            scenario.vehicleInitV = [0;35*mph2mps];
        end
     
    end
    
end

