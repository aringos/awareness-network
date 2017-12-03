classdef scn_RaspPi1080p_LoRa
    
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

        function scenario = scn_RaspPi1080p_LoRa(dt, tend)
            scenario.sensors = ...
               [Sensor('Raspberry_Pi_Camera_1080p30', [20;90],   240*pi/180);...
                Sensor('Raspberry_Pi_Camera_1080p30', [-20;180], -60*pi/180);...
                Sensor('Raspberry_Pi_Camera_1080p30', [20;270],  240*pi/180);...
                Sensor('Raspberry_Pi_Camera_1080p30', [-20;360], -60*pi/180);...
                Sensor('Raspberry_Pi_Camera_1080p30', [20;450],  240*pi/180);...
                Sensor('Raspberry_Pi_Camera_1080p30', [-20;540], -60*pi/180);...
                Sensor('Raspberry_Pi_Camera_1080p30', [20;630],  240*pi/180);...
                Sensor('Raspberry_Pi_Camera_1080p30', [-20;720], -60*pi/180);...
                Sensor('Raspberry_Pi_Camera_1080p30', [20;810],  240*pi/180);...
                Sensor('Raspberry_Pi_Camera_1080p30', [-20;900], -60*pi/180);...
                Sensor('Raspberry_Pi_Camera_1080p30', [20;990],  240*pi/180)];
            
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

