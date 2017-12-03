classdef scn_OV5647_LoRa
    
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

        function scenario = scn_OV5647_LoRa(dt, tend)
            scenario.sensors = ...
               [Sensor('Arducam_OV5647', [20;100],   243*pi/180);...
                Sensor('Arducam_OV5647', [-20;200], -63*pi/180);...
                Sensor('Arducam_OV5647', [20;300],  243*pi/180);...
                Sensor('Arducam_OV5647', [-20;400], -63*pi/180);...
                Sensor('Arducam_OV5647', [20;500],  243*pi/180);...
                Sensor('Arducam_OV5647', [-20;600], -63*pi/180);...
                Sensor('Arducam_OV5647', [20;700],  243*pi/180);...
                Sensor('Arducam_OV5647', [-20;800], -63*pi/180);...
                Sensor('Arducam_OV5647', [20;900],  243*pi/180)];
            
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

