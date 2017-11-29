classdef scn_RaspPi1080p_LoRa
    
    properties
        sensors = [];
        network = 0;
        hardware = 0;
        fusion = 0;
        vehicleAccel = [];
    end
    
    methods

        function scenario = scn_RaspPi1080p_LoRa(dt, tend)
            scenario.sensors = ...
               [Sensor('Raspberry_Pi_Camera_1080p30', [20;50],   240*pi/180); 
                Sensor('Raspberry_Pi_Camera_1080p30', [-20;110], -60*pi/180);
                Sensor('Raspberry_Pi_Camera_1080p30', [20;170],  240*pi/180);
                Sensor('Raspberry_Pi_Camera_1080p30', [-20;230], -60*pi/180);
                Sensor('Raspberry_Pi_Camera_1080p30', [20;290],  240*pi/180);
                Sensor('Raspberry_Pi_Camera_1080p30', [-20;350], -60*pi/180);
                Sensor('Raspberry_Pi_Camera_1080p30', [20;410],  240*pi/180);
                Sensor('Raspberry_Pi_Camera_1080p30', [-20;470], -60*pi/180);
                Sensor('Raspberry_Pi_Camera_1080p30', [20;530],  240*pi/180);
                Sensor('Raspberry_Pi_Camera_1080p30', [-20;590], -60*pi/180);
                Sensor('Raspberry_Pi_Camera_1080p30', [20;650],  240*pi/180)];
            
            scenario.network      = Network('LoRa_RN2483A');
            scenario.hardware     = Hardware('Pi_3');
            scenario.fusion       = FusionCenter();  
            scenario.vehicleAccel = vehicleMotion( 'cruise', dt, tend );    
        end
     
    end
    
end

